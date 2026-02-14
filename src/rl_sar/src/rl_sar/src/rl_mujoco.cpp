#include "rl_mujoco.hpp"

#include <signal.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <mutex>
#include <condition_variable>

static std::mutex g_mtx;
static std::condition_variable g_cv;
static mjModel* g_m = nullptr;
static mjData*  g_d = nullptr;
static bool g_ready = false;

// ============================================================
// Static instance
// ============================================================
RL_Mujoco* RL_Mujoco::instance = nullptr;

// ============================================================
// Ctrl+C handler
// ============================================================
static void signalHandler(int)
{
    if (RL_Mujoco::instance)
        RL_Mujoco::instance->RequestExit_();
}

// ============================================================
// Request exit (UI-safe)
// ============================================================
void RL_Mujoco::RequestExit_()
{
    if (sim)
        sim->exitrequest.store(1);
}

// ============================================================
// ROS spin helper
// ============================================================
//void RL_Mujoco::SpinRosOnce_()
//{
//    if (ros2_node)
//        rclcpp::spin_some(ros2_node);
//}
void RL_Mujoco::SpinRosOnce_()
{
    if (exec_) exec_->spin_some();
}

// ============================================================
// cmd_vel callback
// ============================================================
void RL_Mujoco::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(cmd_vel_mtx_);
    cmd_vel_ = *msg;
    have_cmd_vel_.store(true);
    last_cmd_vel_stamp_ = ros2_node->now();
}

// ============================================================
// Manual keyframe apply (fallback when mj_resetDataKeyframe missing)
// ============================================================
static void ApplyKeyframeManual(mjModel* m, mjData* d, int kf)
{
    if (!m || !d || kf < 0 || kf >= m->nkey) return;

    // key_qpos has size nkey * nq
    if (m->key_qpos)
        mju_copy(d->qpos, m->key_qpos + kf * m->nq, m->nq);

    // key_qvel has size nkey * nv
    if (m->key_qvel)
        mju_copy(d->qvel, m->key_qvel + kf * m->nv, m->nv);

    // key_act has size nkey * na
    if (m->na > 0 && m->key_act)
        mju_copy(d->act, m->key_act + kf * m->na, m->na);

    // Optional: set time to zero-ish
    d->time = 0.0;
}

// ============================================================
// Force stand keyframe at boot
// ============================================================
void RL_Mujoco::ApplyStandKeyframeIfAvailable_()
{
    if (!force_stand_keyframe_ || !mj_model || !mj_data || !sim)
        return;

    std::cout << LOGGER::INFO << "[BOOT] mj_model->nkey=" << mj_model->nkey << "\n";

    const int kf = mj_name2id(mj_model, mjOBJ_KEY, stand_key_name_.c_str());
    if (kf < 0)
    {
        std::cout << LOGGER::WARNING
                  << "[BOOT] Keyframe '" << stand_key_name_
                  << "' not found in MJCF.\n";
        return;
    }

    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    // Try official function if available in your headers/libs.
    // If your build complains "not found", we use manual copy.
#if defined(mjVERSION_HEADER) && (mjVERSION_HEADER >= 230)
    // On certaines versions, c'est bien présent.
    mj_resetDataKeyframe(mj_model, mj_data, kf);
#else
    // Fallback portable
    mj_resetData(mj_model, mj_data);
    ApplyKeyframeManual(mj_model, mj_data, kf);
#endif

    mj_forward(mj_model, mj_data);

    std::cout << LOGGER::INFO << "[BOOT] Applied keyframe '" << stand_key_name_ << "'\n";
}

// ============================================================
// Publish base pose for bridge (throttled)
// ============================================================
void RL_Mujoco::PublishBasePose_()
{
    if (!publish_base_pose_ || !base_pose_pub_ || !mj_data || !ros2_node || !sim)
        return;

    const auto now = ros2_node->now();
    const double period = 1.0 / std::max(1e-6, publish_pose_hz_);

    if ((now - last_pose_pub_stamp_).seconds() < period)
        return;

    last_pose_pub_stamp_ = now;

    double x, y, qw, qx, qy, qz;
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        // freejoint base: qpos[0..6] = x y z qw qx qy qz (dans la plupart des MJCF)
        x  = mj_data->qpos[0];
        y  = mj_data->qpos[1];
        qw = mj_data->qpos[3];
        qx = mj_data->qpos[4];
        qy = mj_data->qpos[5];
        qz = mj_data->qpos[6];
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "odom";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0; // nav2 2D

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;

    base_pose_pub_->publish(pose);
}

// ============================================================
// Constructor
// ============================================================
RL_Mujoco::RL_Mujoco(int argc, char **argv)
{
    instance = this;
    signal(SIGINT, signalHandler);

    // ----------------------------------------------------
    // Args: robot_name scene_name
    // ----------------------------------------------------
    if (argc < 3)
        throw std::runtime_error("Usage: rl_mujoco <robot_name> <scene_name>");

    this->robot_name = argv[1];   // 🔴 CRITIQUE (champ du parent RL)
    this->scene_name = argv[2];
    this->ang_vel_axis = "body";

    std::cout << "[DEBUG] argv[1]        = '" << argv[1] << "'\n";
    std::cout << "[BOOT] robot_name = '" << this->robot_name << "'\n";
    std::cout << "[BOOT] scene_name = '" << this->scene_name << "'\n";
    std::cout.flush();

    // ----------------------------------------------------
    // ROS2 node
    // ----------------------------------------------------
    ros2_node = std::make_shared<rclcpp::Node>("rl_mujoco_node");

    ros2_node->declare_parameter("navigation_mode", true);
    ros2_node->declare_parameter("cmd_vel_timeout_sec", 0.6);
    ros2_node->declare_parameter("cmd_vel_topic", "/cmd_vel");

    navigation_mode_param_ = ros2_node->get_parameter("navigation_mode").as_bool();
    cmd_vel_timeout_sec_   = ros2_node->get_parameter("cmd_vel_timeout_sec").as_double();
    cmd_vel_topic_         = ros2_node->get_parameter("cmd_vel_topic").as_string();

    control.navigation_mode = navigation_mode_param_;
    // init last stamp
    last_cmd_vel_stamp_ = ros2_node->now();
    last_pose_pub_stamp_ = ros2_node->now();

    // Nav2 -> BEST_EFFORT
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    cmd_vel_subscriber__ =
        ros2_node->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, qos,
            std::bind(&RL_Mujoco::CmdvelCallback, this, std::placeholders::_1)
        );

    cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); }
    );
      
    base_pose_pub_ =
        ros2_node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mujoco/base_pose", 10
        );


    // ----------------------------------------------------
    // MuJoCo
    // ----------------------------------------------------
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;

    #if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
    #endif

    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert,
        /* is_passive = */ false
    );


    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR)
        + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/"
        + this->scene_name + ".xml";

    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    this->mj_model = m;
    this->mj_data  = d;

    // joystick (optional)
    this->SetupSysJoystick("/dev/input/js0", 16);

    // ----------------------------------------------------
    // RL init
    // ----------------------------------------------------
    this->ReadYaml(this->robot_name, "base.yaml");

    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // IMPORTANT: pour ton cas, tu peux forcer navigation_mode au démarrage
    // => le robot attend /cmd_vel (Nav2) au lieu du joystick.
    this->control.navigation_mode = true;

    // loops
    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"),
        std::bind(&RL_Mujoco::RobotControl, this)
    );
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Mujoco::RunModel, this)
    );

    this->loop_control->start();
    this->loop_rl->start();

    this->loop_keyboard = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Mujoco::KeyboardInterface, this)
    );
    this->loop_keyboard->start();

    this->loop_joystick = std::make_shared<LoopFunc>(
        "loop_joystick", 0.01, std::bind(&RL_Mujoco::GetSysJoystick, this)
    );
    this->loop_joystick->start();

    // ROS spin loop (évite de bloquer RenderLoop)

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(ros2_node);

    loop_ros_spin = std::make_shared<LoopFunc>(
        "loop_ros_spin", 0.005, std::bind(&RL_Mujoco::SpinRosOnce_, this)
    );
    loop_ros_spin->start();


#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Mujoco::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Mujoco start (ROS2 cmd_vel enabled)" << std::endl;

    // blocking UI loop
    sim->RenderLoop();
}


// ============================================================
// Destructor
// ============================================================

RL_Mujoco::~RL_Mujoco()
{
    instance = nullptr;

    // 1) Stop loops (UNE seule fois chacune)
    if (loop_ros_spin) loop_ros_spin->shutdown();
    if (loop_keyboard) loop_keyboard->shutdown();
    if (loop_joystick) loop_joystick->shutdown();
    if (loop_control)  loop_control->shutdown();
    if (loop_rl)       loop_rl->shutdown();

#ifdef PLOT
    if (loop_plot) loop_plot->shutdown();
#endif

    // 2) Detach ROS executor AFTER spin loop stopped
    if (exec_ && ros2_node)
    {
        // Si tu as exec_mtx_ dans la classe, c’est encore mieux :
        // std::lock_guard<std::mutex> lk(exec_mtx_);
        exec_->remove_node(ros2_node);
    }
    exec_.reset();
    ros2_node.reset();

    std::cout << LOGGER::INFO << "RL_Mujoco exit\n";
}


// ============================================================
// GetState / SetCommand / Forward
// ============================================================
void RL_Mujoco::GetState(RobotState<float> *state)
{
    if (!mj_data) return;

    const int ndof = params.Get<int>("num_of_dofs");

    // IMU layout dépend MJCF (tu as déjà cette logique dans rl_sim_mujoco)
    state->imu.quaternion[0] = mj_data->sensordata[3 * ndof + 0];
    state->imu.quaternion[1] = mj_data->sensordata[3 * ndof + 1];
    state->imu.quaternion[2] = mj_data->sensordata[3 * ndof + 2];
    state->imu.quaternion[3] = mj_data->sensordata[3 * ndof + 3];

    state->imu.gyroscope[0] = mj_data->sensordata[3 * ndof + 4];
    state->imu.gyroscope[1] = mj_data->sensordata[3 * ndof + 5];
    state->imu.gyroscope[2] = mj_data->sensordata[3 * ndof + 6];

    for (int i = 0; i < ndof; ++i)
    {
        const int j = params.Get<std::vector<int>>("joint_mapping")[i];
        state->motor_state.q[i]       = mj_data->sensordata[j];
        state->motor_state.dq[i]      = mj_data->sensordata[j + ndof];
        state->motor_state.tau_est[i] = mj_data->sensordata[j + 2 * ndof];
    }
}

void RL_Mujoco::SetCommand(const RobotCommand<float> *command)
{
    if (!mj_data) return;

    const int ndof = params.Get<int>("num_of_dofs");
    for (int i = 0; i < ndof; ++i)
    {
        const int j = params.Get<std::vector<int>>("joint_mapping")[i];

        mj_data->ctrl[j] =
            command->motor_command.tau[i]
            + command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[j])
            + command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[j + ndof]);
    }
}

std::vector<float> RL_Mujoco::Forward()
{
    std::unique_lock<std::mutex> lock(model_mutex, std::try_to_lock);
    if (!lock.owns_lock())
        return obs.actions;

    std::vector<float> clamped_obs = ComputeObservation();

    std::vector<float> actions;
    if (!params.Get<std::vector<int>>("observations_history").empty())
    {
        history_obs_buf.insert(clamped_obs);
        history_obs = history_obs_buf.get_obs_vec(
            params.Get<std::vector<int>>("observations_history")
        );
        actions = model->forward({history_obs});
    }
    else
    {
        actions = model->forward({clamped_obs});
    }

    if (!params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions,
                     params.Get<std::vector<float>>("clip_actions_lower"),
                     params.Get<std::vector<float>>("clip_actions_upper"));
    }
    return actions;
}

// ============================================================
// RobotControl (FSM + injection cmd_vel + reset/start)
// ============================================================
void RL_Mujoco::RobotControl()
{
    if (!sim || !mj_model || !mj_data) return;

    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    GetState(&robot_state);

    if (control.navigation_mode)
    {
        geometry_msgs::msg::Twist cmd;
        bool ok = false;
        {
            std::lock_guard<std::mutex> lk(cmd_vel_mtx_);
            cmd = cmd_vel_;
            ok = have_cmd_vel_.load();
        }

        const double age = (ros2_node->now() - last_cmd_vel_stamp_).seconds();
        if (!ok || age > cmd_vel_timeout_sec_)
        {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
        }

        control.x   = static_cast<float>(cmd.linear.x);
        control.y   = static_cast<float>(cmd.linear.y);
        control.yaw = static_cast<float>(cmd.angular.z);
    }

    StateController(&robot_state, &robot_command);

    // Reset
    if (control.current_keyboard == Input::Keyboard::R ||
        control.current_gamepad  == Input::Gamepad::RB_Y)
    {
        mj_resetData(mj_model, mj_data);
        mj_forward(mj_model, mj_data);
        ApplyStandKeyframeIfAvailable_();
    }

    // Start/Stop
    if (control.current_keyboard == Input::Keyboard::Enter ||
        control.current_gamepad  == Input::Gamepad::RB_X)
    {
        sim->run = sim->run ? 0 : 1;
        simulation_running = (sim->run != 0);
    }

    // Log qpos[0..6]
    static std::ofstream f("/tmp/qpos_log.txt");
    static int k = 0;
    if (f && (++k % 200) == 0)
    {
        f << mj_data->time << " "
          << mj_data->qpos[0] << " " << mj_data->qpos[1] << " " << mj_data->qpos[2] << " "
          << mj_data->qpos[3] << " " << mj_data->qpos[4] << " " << mj_data->qpos[5] << " " << mj_data->qpos[6]
          << "\n";
        f.flush();
    }

    if (this->control.current_keyboard == Input::Keyboard::P)  // choisis une touche dispo
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

        std::ofstream f("/tmp/qpos_after_manual_stand.txt");
        f << std::setprecision(17);

        f << "time " << mj_data->time << "\n";
        f << "nq " << mj_model->nq << "\n";
        for (int i = 0; i < mj_model->nq; ++i)
            f << i << " " << mj_data->qpos[i] << "\n";

        f.close();
        std::cout << "[DUMP] Saved /tmp/qpos_after_manual_stand.txt\n";
    }

    SetCommand(&robot_command);
     control.ClearInput(); // évite re-trigger
}

// ============================================================
// RunModel (policy + queues + publish pose)
// ============================================================
void RL_Mujoco::RunModel()
{
    if (rl_init_done && simulation_running)
    {
        episode_length_buf += 1;

        obs.ang_vel   = robot_state.imu.gyroscope;
        obs.base_quat = robot_state.imu.quaternion;
        obs.dof_pos   = robot_state.motor_state.q;
        obs.dof_vel   = robot_state.motor_state.dq;
        obs.commands  = {control.x, control.y, control.yaw};

        obs.actions = Forward();

        ComputeOutput(obs.actions, output_dof_pos, output_dof_vel, output_dof_tau);

        if (!output_dof_pos.empty()) output_dof_pos_queue.push(output_dof_pos);
        if (!output_dof_vel.empty()) output_dof_vel_queue.push(output_dof_vel);
        if (!output_dof_tau.empty()) output_dof_tau_queue.push(output_dof_tau);
    }

    PublishBasePose_();
}

// ============================================================
// Joystick
// ============================================================
void RL_Mujoco::SetupSysJoystick(const std::string& device, int bits)
{
    sys_js = std::make_unique<Joystick>(device);
    if (!sys_js->isFound())
    {
        std::cout << LOGGER::WARNING << "Joystick [" << device << "] not found.\n";
        return;
    }
    sys_js_max_value = (1 << (bits - 1));
}

void RL_Mujoco::GetSysJoystick()
{
    for (int i = 0; i < 20; ++i)
    {
        sys_js_button[i].on_press = false;
        sys_js_button[i].on_release = false;
    }

    if (!sys_js) return;

    while (sys_js->sample(&sys_js_event))
    {
        if (sys_js_event.isButton())
            sys_js_button[sys_js_event.number].update(sys_js_event.value);
        else if (sys_js_event.isAxis())
        {
            double normalized = double(sys_js_event.value) / sys_js_max_value;
            if (std::abs(normalized) < axis_deadzone)
                sys_js_axis[sys_js_event.number] = 0;
            else
                sys_js_axis[sys_js_event.number] = sys_js_event.value;
        }
    }

    // map buttons
    if (sys_js_button[0].on_press) control.SetGamepad(Input::Gamepad::A);
    if (sys_js_button[1].on_press) control.SetGamepad(Input::Gamepad::B);
    if (sys_js_button[2].on_press) control.SetGamepad(Input::Gamepad::X);
    if (sys_js_button[3].on_press) control.SetGamepad(Input::Gamepad::Y);
    if (sys_js_button[4].on_press) control.SetGamepad(Input::Gamepad::LB);
    if (sys_js_button[5].on_press) control.SetGamepad(Input::Gamepad::RB);

    float ly = -float(sys_js_axis[1]) / float(sys_js_max_value);
    float lx = -float(sys_js_axis[0]) / float(sys_js_max_value);
    float rx = -float(sys_js_axis[3]) / float(sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    // joystick override only if NOT navigation_mode
    if (has_input && !control.navigation_mode)
    {
        control.x = ly;
        control.y = lx;
        control.yaw = rx;
        sys_js_active = true;
    }
    else if (sys_js_active && !control.navigation_mode)
    {
        control.x = 0.f;
        control.y = 0.f;
        control.yaw = 0.f;
        sys_js_active = false;
    }
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        RL_Mujoco app(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << LOGGER::ERROR << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
