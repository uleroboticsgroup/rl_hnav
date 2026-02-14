/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_mujoco.hpp"

RL_Mujoco* RL_Mujoco::instance = nullptr;

// Signal handler for Ctrl+C
static void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Mujoco::instance && RL_Mujoco::instance->sim)
    {
        RL_Mujoco::instance->sim->exitrequest.store(1);
    }
    // ROS2 shutdown handled in main
}

RL_Mujoco::RL_Mujoco(int argc, char **argv)
{
    instance = this;

    // Args: robot_name scene_name
    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    this->robot_name = argv[1];
    this->scene_name = argv[2];
    this->ang_vel_axis = "body";

    // ROS2 init node (non-bloquant, on spin dans un loop)
    ros2_node = std::make_shared<rclcpp::Node>("rl_mujoco_node");

    // Declare params (so --ros-args -p ... works)
    ros2_node->declare_parameter<bool>("navigation_mode", true);
    ros2_node->declare_parameter<double>("cmd_vel_timeout_sec", 0.6);
    ros2_node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // Read params
    navigation_mode_param_ = ros2_node->get_parameter("navigation_mode").as_bool();
    cmd_vel_timeout_sec_   = ros2_node->get_parameter("cmd_vel_timeout_sec").as_double();
    cmd_vel_topic_         = ros2_node->get_parameter("cmd_vel_topic").as_string();

    std::cout << LOGGER::INFO
            << "[ROS2] navigation_mode=" << (navigation_mode_param_ ? "true" : "false")
            << " cmd_vel_timeout_sec=" << cmd_vel_timeout_sec_
            << " cmd_vel_topic=" << cmd_vel_topic_
            << std::endl;

    // Apply to control
    this->control.navigation_mode = navigation_mode_param_;

    // Subscribe to cmd_vel_topic  (Nav2 -> here)
    cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); }
    );
        
    //rclcpp::QoS qos(rclcpp::KeepLast(10));
    //qos.reliable();
    //cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
    //    cmd_vel_topic_, qos, std::bind(&RL_Mujoco::CmdvelCallback, this, std::placeholders::_1)
    //);

    // Publish base_pose_pub_
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;
    base_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mujoco/base_pose", 10
    );


    // init last stamp
    last_cmd_vel_stamp_ = ros2_node->now();

    // ---- MuJoCo launch (copié de rl_sim_mujoco) ----
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
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
        &cam, &opt, &pert, /* is_passive = */ false);

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

    // ---- RL init (copié de rl_sim / rl_sim_mujoco) ----
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
    this->loop_ros_spin = std::make_shared<LoopFunc>(
        "loop_ros_spin", 0.005, std::bind(&RL_Mujoco::SpinRosOnce_, this)
    );
    this->loop_ros_spin->start();

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

RL_Mujoco::~RL_Mujoco()
{
    instance = nullptr;

    if (this->loop_ros_spin) this->loop_ros_spin->shutdown();
    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_joystick) this->loop_joystick->shutdown();
    if (this->loop_control)  this->loop_control->shutdown();
    if (this->loop_rl)       this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot)     this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Mujoco exit" << std::endl;
}

void RL_Mujoco::SpinRosOnce_()
{
    if (!rclcpp::ok()) return;
    rclcpp::spin_some(ros2_node);
}

void RL_Mujoco::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(cmd_vel_mtx_);
    cmd_vel_ = *msg;
    have_cmd_vel_.store(true);
    last_cmd_vel_stamp_ = ros2_node->now();
}

void RL_Mujoco::GetState(RobotState<float> *state)
{
    if (!mj_data) return;

    // Same mapping as rl_sim_mujoco
    state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
    state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
    state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
    state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

    state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
    state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
    state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const int idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
        state->motor_state.q[i] = mj_data->sensordata[idx];
        state->motor_state.dq[i] = mj_data->sensordata[idx + this->params.Get<int>("num_of_dofs")];
        state->motor_state.tau_est[i] = mj_data->sensordata[idx + 2 * this->params.Get<int>("num_of_dofs")];
    }
}

void RL_Mujoco::SetCommand(const RobotCommand<float> *command)
{
    if (!mj_data) return;

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const int idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
        mj_data->ctrl[idx] =
            command->motor_command.tau[i] +
            command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[idx]) +
            command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[idx + this->params.Get<int>("num_of_dofs")]);
    }
}

void RL_Mujoco::RobotControl()
{
    // Prevent race conditions with MuJoCo physics thread
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);

    // Let FSM decide posture/standup/ready
    this->StateController(&this->robot_state, &this->robot_command);

    // reset
    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
        }
    }

    // pause/resume sim
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

void RL_Mujoco::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;

        this->obs.ang_vel   = this->robot_state.imu.gyroscope;
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos   = this->robot_state.motor_state.q;
        this->obs.dof_vel   = this->robot_state.motor_state.dq;

        // Default: joystick/keyboard control
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};

        // Navigation mode: cmd_vel overrides (same logic as rl_sim ROS)

        if (this->control.navigation_mode)
        {
            geometry_msgs::msg::Twist cmd;
            rclcpp::Time stamp;
            {
                std::lock_guard<std::mutex> lk(cmd_vel_mtx_);
                cmd = cmd_vel_;
                stamp = last_cmd_vel_stamp_;
            }

            const double age = (ros2_node->now() - stamp).seconds();

            if (!have_cmd_vel_.load() || age > cmd_vel_timeout_sec_)
            {
                // timeout => stop
                this->obs.commands = {0.0f, 0.0f, 0.0f};
            }
            else
            {
                this->obs.commands = {(float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.angular.z};
            }
        }


        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty()) output_dof_pos_queue.push(this->output_dof_pos);
        if (!this->output_dof_vel.empty()) output_dof_vel_queue.push(this->output_dof_vel);
        if (!this->output_dof_tau.empty()) output_dof_tau_queue.push(this->output_dof_tau);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now();
        pose.header.frame_id = "odom";   // IMPORTANT (pas map)

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = qw;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;

        base_pose_pub_->publish(pose);


        // Optional protections
        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);
    }
}

std::vector<float> RL_Mujoco::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions,
                     this->params.Get<std::vector<float>>("clip_actions_lower"),
                     this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    return actions;
}

void RL_Mujoco::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // keep running without joystick
    }
    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Mujoco::GetSysJoystick()
{
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    if (!this->sys_js) return;

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
                this->sys_js_axis[this->sys_js_event.number] = 0;
            else
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
        }
    }

    // Map only if you still want manual override.
    // If you want "Nav2 only", you can skip updating control.x/y/yaw when navigation_mode is true.
    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (!this->control.navigation_mode)  // manual only if not nav mode
    {
        if (has_input)
        {
            this->control.x = ly;
            this->control.y = lx;
            this->control.yaw = rx;
            this->sys_js_active = true;
        }
        else if (this->sys_js_active)
        {
            this->control.x = 0.0f;
            this->control.y = 0.0f;
            this->control.yaw = 0.0f;
            this->sys_js_active = false;
        }
    }
}

void RL_Mujoco::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.01);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    {
        RL_Mujoco rl(argc, argv);
        // ctor blocks in RenderLoop()
    }
    rclcpp::shutdown();
    return 0;
}
