/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 *
 * RL_Real_G1_EDU23.cpp
 * ----------------------------------------
 * A safe EDU+ (23 active joints) adapter that keeps a 29-DoF policy interface.
 *
 * Key goals:
 *  1) Allow testing cmd_vel reception (Nav2 / teleop) without touching hardware.
 *  2) Allow testing the whole RL pipeline using a FAKE LowState (no robot required).
 *  3) Allow running on the real robot while masking missing joints (EDU23).
 *
 * IMPORTANT DESIGN NOTE ABOUT YAML:
 *  - base.yaml (g1) is loaded in the constructor, then later your FSM Enter()
 *    calls rl.InitRL("g1/robomimic/locomotion"), which loads policy YAML.
 *  - That policy YAML overwrites rl.params.config_node (because that's how rl_sar is written).
 *  - To avoid "policy YAML breaking hardware binding", this file provides a "base snapshot"
 *    (base_snap_) of critical keys loaded from base.yaml.
 *
 * What we do here:
 *  - For REAL hardware I/O, we do NOT rely on "joint_names/controller_names" from policy.
 *  - For mapping policy index -> hardware motor_state index, we use the *current* rl.params joint_mapping
 *    (this must be the policy joint_mapping from policy YAML).
 *  - If you want "policy YAML not to overwrite base.yaml keys", the clean way is to do a merge
 *    in your FSM Enter() (restore base keys + keep policy-only keys). But even without that merge,
 *    the code below remains safe because:
 *       - GetState_real() initializes all 29 DoF values with defaults.
 *       - Missing joints are never read/written to hardware.
 *       - SetCommand() disables everything then enables only active joints.
 *
 * ----------------------------------------
 * Build-time switches:
 *
 *   -DUSE_ROS -DUSE_ROS2
 *       Enable ROS2 cmd_vel subscriber.
 *
 *   -DFAKE_LOWSTATE_TEST
 *       Use fake lowstate values (no DDS subscription needed to test pipeline).
 *
 *   -DDRY_RUN_NO_LOW_CMD
 *       Never publish LowCmd even on real robot. Good for first safety tests.
 *
 *   -DCMD_VEL_ONLY
 *       Only print cmd_vel reception; do not run RL / do not publish LowCmd.
 * ----------------------------------------
 */

#include "rl_real_g1_edu23.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

// ----------------------------
// Helper: clamp
// ----------------------------
static inline float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}
static_assert(is_edu23_active_joint(0) == true, "EDU23 joint mask broken");
static_assert(is_edu23_active_joint(14) == false, "EDU23 joint mask broken");

// ----------------------------
// Constructor
// ----------------------------
RL_Real_G1_EDU23::RL_Real_G1_EDU23(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, &RL_Real_G1_EDU23::CmdvelCallback, this);

#elif defined(USE_ROS2) && defined(USE_ROS)

    ros2_node = std::make_shared<rclcpp::Node>("rl_real_g1_edu23_node");

    const int hw_mode_param = ros2_node->declare_parameter<int>("hw_mode", 0);
    if (hw_mode_param < 0 || hw_mode_param > 3) {
    throw std::runtime_error("Invalid hw_mode param (expected 0..3)");
    }
    hw_mode_ = static_cast<HwMode>(hw_mode_param);


    publish_lowcmd_ = ros2_node->declare_parameter<bool>("publish_lowcmd", true);
    fake_rl_full_   = ros2_node->declare_parameter<bool>("fake_rl_full", false);
    cmd_vel_only_   = ros2_node->declare_parameter<bool>("cmd_vel_only", false);

    RCLCPP_INFO(
        ros2_node->get_logger(),
        "HW MODE = %d (0=REAL,1=FAKE,2=CMD_VEL_ONLY,3=DRY_RUN)",
        hw_mode_param
    );

    if (hw_mode_ == HwMode::REAL)
    {
        #ifndef ENABLE_REAL_ROBOT
            throw std::runtime_error(
                "REAL robot mode requested but ENABLE_REAL_ROBOT not defined at compile-time"
            );
        #endif
    }

    // ---- parameters (ROS2) ----
    cmd_vel_timeout_sec_   = ros2_node->declare_parameter<double>("cmd_vel_timeout_sec", 0.6);
    cmd_vel_topic_         = ros2_node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    navigation_mode_param_ = ros2_node->declare_parameter<bool>("navigation_mode", false);
    navigation_mode_force_ = ros2_node->declare_parameter<bool>("navigation_mode_force", true);

    RCLCPP_INFO(ros2_node->get_logger(), "navigation_mode_force=%d", (int)navigation_mode_force_);


    // Apply initial navigation mode to your control state
    this->control.navigation_mode = navigation_mode_param_;

    // init last cmd time to "now" so we don't instantly timeout at startup
    last_cmd_vel_time_ = ros2_node->now();

    // ---- cmd_vel subscription (single) ----
    using std::placeholders::_1;
    cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RL_Real_G1_EDU23::CmdvelCallback, this, _1)
    );


#endif


    // ----------------------------------------
    // 1) Load base.yaml
    // ----------------------------------------
    this->ang_vel_axis = "body";
    this->robot_name = "g1";
    this->ReadYaml(this->robot_name, "base.yaml");

    // Take a snapshot of base.yaml keys (optional safety / future merge)
    base_snap_.dt                 = this->params.Get<float>("dt", 0.005f);
    base_snap_.decimation         = this->params.Get<int>("decimation", 4);
    base_snap_.num_of_dofs        = this->params.Get<int>("num_of_dofs", 29);
    base_snap_.joint_mapping      = this->params.Get<std::vector<int>>("joint_mapping", {});
    base_snap_.default_dof_pos    = this->params.Get<std::vector<float>>("default_dof_pos", {});
    base_snap_.torque_limits      = this->params.Get<std::vector<float>>("torque_limits", {});
    base_snap_.joint_names        = this->params.Get<std::vector<std::string>>("joint_names", {});
    base_snap_.joint_controller_names = this->params.Get<std::vector<std::string>>("joint_controller_names", {});
    std::cout << "[INFO] Base hardware YAML snapshot saved\n";

    // Debug
    {
        auto jm = this->params.Get<std::vector<int>>("joint_mapping");
        std::cout << "[DEBUG] joint_mapping AFTER base.yaml size=" << jm.size() << " : ";
        for (int i = 0; i < std::min<int>((int)jm.size(), 12); ++i)
            std::cout << jm[i] << " ";
        std::cout << "...\n";
    }

    // ----------------------------------------
    // 2) FSM creation
    // ----------------------------------------
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr) this->fsm = *fsm_ptr;
    }
    else
    {
        std::cout << LOGGER::ERROR
                  << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // ----------------------------------------
    // 3) Init robot control containers (policy-space = 29)
    // ----------------------------------------
    this->mode_pr = Mode::PR;
    this->mode_machine = 0;

    this->InitLowCmd(); // SAFE: everything disabled

    // Keep policy-space size (29). This must match your policy model.
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // ----------------------------------------
    // 4) MotionSwitcher (Unitree)
    // ----------------------------------------
    this->msc.SetTimeout(5.0f);
    this->msc.Init();

    // Shut down any other motion control service (recommended by Unitree examples)
    std::string form, name;
    const int max_attempts = 10;
    int attempts = 0;

    while (this->msc.CheckMode(form, name), !name.empty())
    {
        if (this->msc.ReleaseMode())
            break;

        if (++attempts >= max_attempts)
        {
            std::cerr << "[FATAL] Unable to release motion mode after "
                    << max_attempts << " attempts" << std::endl;
            break;
        }

        sleep(3);
    }


    // ----------------------------------------
    // 5) DDS Pub/Sub
    // ----------------------------------------
    this->lowcmd_publisher.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    this->lowcmd_publisher->InitChannel();

    this->lowstate_subscriber.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    this->lowstate_subscriber->InitChannel(
        std::bind(&RL_Real_G1_EDU23::LowStateHandler, this, std::placeholders::_1), 1);

    this->imutorso_subscriber.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    this->imutorso_subscriber->InitChannel(
        std::bind(&RL_Real_G1_EDU23::ImuTorsoHandler, this, std::placeholders::_1), 1);

    // ----------------------------------------
    // 6) Loops
    // ----------------------------------------
    this->loop_keyboard = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Real_G1_EDU23::KeyboardInterface, this));

    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"),
        std::bind(&RL_Real_G1_EDU23::RobotControl, this));

    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real_G1_EDU23::RunModel, this));

    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &v : this->plot_real_joint_pos) v = std::vector<float>(this->plot_size, 0);
    for (auto &v : this->plot_target_joint_pos) v = std::vector<float>(this->plot_size, 0);
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real_G1_EDU23::Plot, this));
    this->loop_plot->start();
#endif

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO
              << "RL_Real_G1_EDU23 started (policy=29dof, robot=EDU+ 23 active joints)" << std::endl;
}

RL_Real_G1_EDU23::~RL_Real_G1_EDU23()
{
    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control)  this->loop_control->shutdown();
    if (this->loop_rl)       this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot)     this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real_G1_EDU23 exit" << std::endl;
}

// ----------------------------
// DDS callbacks
// ----------------------------
void RL_Real_G1_EDU23::LowStateHandler(const void *message)
{
    this->unitree_low_state = *(const LowState_ *)message;
    this->got_lowstate.store(true, std::memory_order_relaxed);
}

void RL_Real_G1_EDU23::ImuTorsoHandler(const void *message)
{
    this->unitree_imu_torso = *(const IMUState_ *)message;
}

// ----------------------------
// GetState dispatch
// ----------------------------

void RL_Real_G1_EDU23::GetState(RobotState<float> *state)
{
    switch (hw_mode_)
    {
        case HwMode::FAKE_LOWSTATE:
            GetState_test(state);
            break;

        case HwMode::REAL:
            GetState_real(state);
            break;

        default:
            // CMD_VEL_ONLY / DRY_RUN should never reach here
            break;
    }
}


// ----------------------------
// GetState_real (EDU23 mask applied)
// ----------------------------
void RL_Real_G1_EDU23::GetState_real(RobotState<float> *state)
{
#ifdef CMD_VEL_ONLY
    // Minimal state; do not read hardware aggressively.
    state->imu.quaternion = {1.f, 0.f, 0.f, 0.f};
    state->imu.gyroscope  = {0.f, 0.f, 0.f};
    const int ndof = this->params.Get<int>("num_of_dofs", 29);
    state->motor_state.resize(ndof);
    for (int i = 0; i < ndof; ++i) {
        state->motor_state.q[i] = 0.f;
        state->motor_state.dq[i] = 0.f;
        state->motor_state.tau_est[i] = 0.f;
    }
    return;
#endif

    // If LowState has not arrived yet, return safe defaults.
    if (!this->got_lowstate.load(std::memory_order_relaxed))
    {
        const int ndof = this->params.Get<int>("num_of_dofs", 29);
        const auto def = this->params.Get<std::vector<float>>("default_dof_pos", {});
        state->motor_state.resize(ndof);

        state->imu.quaternion = {1.f, 0.f, 0.f, 0.f};
        state->imu.gyroscope  = {0.f, 0.f, 0.f};

        for (int i = 0; i < ndof; ++i)
        {
            state->motor_state.q[i]       = (i < (int)def.size()) ? def[i] : 0.f;
            state->motor_state.dq[i]      = 0.f;
            state->motor_state.tau_est[i] = 0.f;
        }
        return;
    }

    // Update robot type once
    if (this->mode_machine != this->unitree_low_state.mode_machine())
    {
        static bool printed_type = false;
        if (!printed_type) {
            std::cout << "G1 type: " << unsigned(this->unitree_low_state.mode_machine()) << std::endl;
            printed_type = true;
        }
        this->mode_machine = this->unitree_low_state.mode_machine();

    }

    
    // Gamepad parse
    //std::memcpy(this->remote_data_rx.buff, &unitree_low_state.wireless_remote()[0], 40);
    std::memcpy(this->remote_data_rx.buff,
            &this->unitree_low_state.wireless_remote()[0],
            40);
    this->gamepad.update(this->remote_data_rx.RF_RX);

    // Map buttons (unchanged behavior)
    if (this->gamepad.A.pressed) this->control.SetGamepad(Input::Gamepad::A);
    if (this->gamepad.B.pressed) this->control.SetGamepad(Input::Gamepad::B);
    if (this->gamepad.X.pressed) this->control.SetGamepad(Input::Gamepad::X);
    if (this->gamepad.Y.pressed) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->gamepad.R1.pressed) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->gamepad.L1.pressed) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->gamepad.F1.pressed) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->gamepad.F2.pressed) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->gamepad.up.pressed) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->gamepad.down.pressed) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->gamepad.left.pressed) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->gamepad.right.pressed) this->control.SetGamepad(Input::Gamepad::DPadRight);

    if (this->gamepad.L1.pressed && this->gamepad.A.pressed) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->gamepad.L1.pressed && this->gamepad.B.pressed) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->gamepad.L1.pressed && this->gamepad.X.pressed) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->gamepad.L1.pressed && this->gamepad.Y.pressed) this->control.SetGamepad(Input::Gamepad::LB_Y);

    if (this->gamepad.R1.pressed && this->gamepad.A.pressed) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->gamepad.R1.pressed && this->gamepad.B.pressed) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->gamepad.R1.pressed && this->gamepad.X.pressed) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->gamepad.R1.pressed && this->gamepad.Y.pressed) this->control.SetGamepad(Input::Gamepad::RB_Y);

    if (this->gamepad.L1.pressed && this->gamepad.R1.pressed) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // Joystick continuous
    this->control.x = this->gamepad.ly;
    this->control.y = -this->gamepad.lx;
    this->control.yaw = -this->gamepad.rx;

    // IMU: use low_state IMU by default.
    // If you want torso IMU, you can switch to unitree_imu_torso.* fields here.
    state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0];
    state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1];
    state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2];
    state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3];
    for (int i = 0; i < 3; ++i)
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];

    // ---- EDU23-safe motor_state build ----
    const int ndof = this->params.Get<int>("num_of_dofs", 29); // policy space
    const auto jm_policy  = this->params.Get<std::vector<int>>("joint_mapping", {});
    const auto def        = this->params.Get<std::vector<float>>("default_dof_pos", {});

    state->motor_state.resize(ndof);

    // 1) initialize with defaults so missing joints have stable values
    for (int i = 0; i < ndof; ++i)
    {
        state->motor_state.q[i]       = (i < (int)def.size()) ? def[i] : 0.0f;
        state->motor_state.dq[i]      = 0.0f;
        state->motor_state.tau_est[i] = 0.0f;
    }

    // 2) overwrite only joints that exist physically on EDU+
    const int ms_size = (int)this->unitree_low_state.motor_state().size(); // e.g., 35
    for (int i = 0; i < ndof; ++i)
    {
        if (!is_edu23_active_joint(i))
            continue;

        const int idx = (i < (int)jm_policy.size()) ? jm_policy[i] : i;
        if (idx < 0 || idx >= ms_size)
            continue;

        state->motor_state.q[i]       = this->unitree_low_state.motor_state()[idx].q();
        state->motor_state.dq[i]      = this->unitree_low_state.motor_state()[idx].dq();
        state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[idx].tau_est();
    }
}

// ----------------------------
// GetState_test (FAKE_LOWSTATE_TEST)
// ----------------------------
void RL_Real_G1_EDU23::GetState_test(RobotState<float> *state)
{
    const int ndof = this->params.Get<int>("num_of_dofs", 29);
    const auto jm_policy = this->params.Get<std::vector<int>>("joint_mapping", {});
    state->motor_state.resize(ndof);

    // Initialize fake hw arrays once
    static const int hw_size = 35;

    if (!fake_inited)
    {
        fake_inited = true;
        fake_hw_q.resize(hw_size);
        fake_hw_dq.resize(hw_size);
        fake_hw_tau.resize(hw_size);

        for (int k = 0; k < hw_size; ++k)
        {
            fake_hw_q[k]   = 1000.f + k;
            fake_hw_dq[k]  = 2000.f + k;
            fake_hw_tau[k] = 3000.f + k;
        }

        //this->got_lowstate.store(true);
        this->got_lowstate.store(true, std::memory_order_relaxed);

        std::cout << "[FAKE_LOWSTATE_TEST] Synthetic LowState ready (hw_size=" << hw_size << ")\n";
    }

    // IMU stable
    state->imu.quaternion = {1.f, 0.f, 0.f, 0.f};
    state->imu.gyroscope  = {0.f, 0.f, 0.f};

    // Fill motor_state using policy mapping: policy_i -> hw index jm_policy[i]
    for (int i = 0; i < ndof; ++i)
    {
        int hw = (i < (int)jm_policy.size()) ? jm_policy[i] : -1;
        if (hw < 0 || hw >= hw_size)
        {
            state->motor_state.q[i]       = 0.f;
            state->motor_state.dq[i]      = 0.f;
            state->motor_state.tau_est[i] = 0.f;
        }
        else
        {
            state->motor_state.q[i]       = fake_hw_q[hw];
            state->motor_state.dq[i]      = fake_hw_dq[hw];
            state->motor_state.tau_est[i] = fake_hw_tau[hw];
        }
    }

    // Print mapping check only once
    static bool printed = false;
    if (!printed)
    {
        printed = true;
        std::cout << "\n===== TEST: policy->HW mapping check =====\n";
        std::cout << "i | jm_policy[i] | expected_q | got_q\n";
        for (int i = 0; i < std::min(ndof, 29); ++i)
        {
            int hw = (i < (int)jm_policy.size()) ? jm_policy[i] : -1;
            float expected = (hw >= 0) ? (1000.0f + hw) : -1.0f;
            float got = state->motor_state.q[i];
            std::cout << i << " | " << hw << " | " << expected << " | " << got << "\n";
        }
        std::cout << "=========================================\n\n";
    }
}

// ----------------------------
// SetCommand (EDU23 mask applied)
// ----------------------------
void RL_Real_G1_EDU23::SetCommand(const RobotCommand<float>* command)
{
    // ----------------------------
    // Safety guards (runtime only)
    // ----------------------------
    if (!command)
        return;

    if (hw_mode_ != HwMode::REAL)
    {
        // FAKE_LOWSTATE / CMD_VEL_ONLY / DRY_RUN
        // → Never publish LowCmd
        return;
    }

    // ----------------------------
    // Real robot command path
    // ----------------------------
    this->unitree_low_command.mode_pr() = static_cast<uint8_t>(this->mode_pr);
    this->unitree_low_command.mode_machine() = this->mode_machine;

    const int ndof = this->params.Get<int>("num_of_dofs", 29);
    const auto jm_policy = this->params.Get<std::vector<int>>("joint_mapping", {});

    auto& mc = this->unitree_low_command.motor_cmd();
    const int mc_size = static_cast<int>(mc.size());

    // 0) SAFE: disable everything first
    for (int k = 0; k < mc_size; ++k)
    {
        mc[k].mode() = 0;
        mc[k].q()    = 0;
        mc[k].dq()   = 0;
        mc[k].kp()   = 0;
        mc[k].kd()   = 0;
        mc[k].tau()  = 0;
    }

    // 1) Enable only EDU23 active joints
    for (int i = 0; i < ndof; ++i)
    {
        if (!is_edu23_active_joint(i))
            continue;

        const int idx = (i < (int)jm_policy.size()) ? jm_policy[i] : i;
        if (idx < 0 || idx >= mc_size)
            continue;

        mc[idx].mode() = 1;
        mc[idx].q()    = command->motor_command.q[i];
        mc[idx].dq()   = command->motor_command.dq[i];
        mc[idx].kp()   = command->motor_command.kp[i];
        mc[idx].kd()   = command->motor_command.kd[i];
        mc[idx].tau()  = command->motor_command.tau[i];
    }

    // 2) CRC + publish
    this->unitree_low_command.crc() =
        Crc32Core(
            reinterpret_cast<uint32_t*>(&unitree_low_command),
            (sizeof(LowCmd_) >> 2) - 1
        );

    lowcmd_publisher->Write(unitree_low_command);
}


// ----------------------------
// RobotControl loop
// ----------------------------
void RL_Real_G1_EDU23::RobotControl()
{
#ifdef CMD_VEL_ONLY
    return;
#endif

    // Runtime guards (if you support hw_mode_ without rebuild)
    if (hw_mode_ == HwMode::CMD_VEL_ONLY) return;

    this->GetState(&this->robot_state);

#if defined(USE_ROS2) && defined(USE_ROS)
    RCLCPP_INFO_THROTTLE(
        ros2_node->get_logger(), *ros2_node->get_clock(), 1000 /*ms*/,
        "[DBG] got_lowstate=%d", (int)this->got_lowstate.load());
#else
    static int cnt = 0;
    if ((cnt++ % 200) == 0)
        std::cout << "[DBG] got_lowstate=" << (int)this->got_lowstate.load() << "\n";
#endif

    static bool warned = false;

    // Only block on LowState when REAL
    if (hw_mode_ == HwMode::REAL &&
        !this->got_lowstate.load(std::memory_order_relaxed))
    {
        if (!warned)
        {
            std::cout << "[WAIT] no LowState yet\n";
            warned = true;
        }
        return;
    }
    warned = false;

    this->StateController(&this->robot_state, &this->robot_command);
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);  // SetCommand already blocks DRY_RUN/FAKE etc.
}

// ------------------------------------------------------------------
// Helper (put in .cpp, and declare in header as:
// bool GetFreshCmdVel(float& vx, float& vy, float& wz) const; )
// ------------------------------------------------------------------
bool RL_Real_G1_EDU23::GetFreshCmdVel(float &vx, float &vy, float &wz) const
{
#if defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cv;
    rclcpp::Time last_t;
    {
        std::lock_guard<std::mutex> lk(this->cmd_vel_mtx_);
        cv     = this->cmd_vel_; 
        last_t = this->last_cmd_vel_time_;
    }

    if (last_t.nanoseconds() == 0) return false;

    const auto now = ros2_node->now();
    const double age = (now - last_t).seconds();
    if (age > this->cmd_vel_timeout_sec_) return false;

    vx = static_cast<float>(cv.linear.x);
    vy = static_cast<float>(cv.linear.y);
    wz = static_cast<float>(cv.angular.z);
    return true;

#elif defined(USE_ROS1) && defined(USE_ROS)
    vx = static_cast<float>(this->cmd_vel.linear.x);
    vy = static_cast<float>(this->cmd_vel.linear.y);
    wz = static_cast<float>(this->cmd_vel.angular.z);
    return true;
#else
    (void)vx; (void)vy; (void)wz;
    return false;
#endif
}


// ------------------------------------------------------------------
// RunModel() (drop-in replacement)
// ------------------------------------------------------------------
void RL_Real_G1_EDU23::RunModel()
{
    // ----------------------------
    // Fast exits
    // ----------------------------
    if (this->cmd_vel_only_) return;
    if (!this->rl_init_done) return;
    if (hw_mode_ == HwMode::FAKE_LOWSTATE && !fake_rl_full_) return;

    // ----------------------------
    // 1) Update episode + base obs
    // ----------------------------
    this->episode_length_buf += 1;

    this->obs.ang_vel   = this->robot_state.imu.gyroscope;
    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos   = this->robot_state.motor_state.q;
    this->obs.dof_vel   = this->robot_state.motor_state.dq;

    // Default (gamepad/joystick)
    float vx = this->control.x;
    float vy = this->control.y;
    float wz = this->control.yaw;

    // ----------------------------
    // 2) Optional nav mode => /cmd_vel
    // ----------------------------
#if defined(USE_ROS) && (defined(USE_ROS1) || defined(USE_ROS2))

    const bool nav_on =
#if defined(USE_ROS2)
        (navigation_mode_force_ ? true : this->control.navigation_mode);
#else
        this->control.navigation_mode;
#endif

    if (nav_on)
    {
        float vxf = 0.f, vyf = 0.f, wzf = 0.f;
        const bool fresh = GetFreshCmdVel(vxf, vyf, wzf);

        if (fresh)
        {
            vx = vxf; vy = vyf; wz = wzf;
        }
        else
        {
            vx = 0.f; vy = 0.f; wz = 0.f;
        }

        // ✅ Keep everything consistent (FSM/debug/other code paths)
        this->control.x   = vx;
        this->control.y   = vy;
        this->control.yaw = wz;

#if defined(USE_ROS2)
        const double age_dbg =
            (this->last_cmd_vel_time_.nanoseconds() == 0)
                ? -1.0
                : (ros2_node->now() - this->last_cmd_vel_time_).seconds();

        RCLCPP_INFO_THROTTLE(
            ros2_node->get_logger(), *ros2_node->get_clock(), 500 /*ms*/,
            "[NAV] nav_on=%d fresh=%d age=%.3fs cmd=[%.2f %.2f %.2f]",
            (int)nav_on, (int)fresh, age_dbg, vx, vy, wz
        );
#endif
    }

#endif // USE_ROS

    // ----------------------------
    // 3) Feed policy
    // ----------------------------
    this->obs.commands = {vx, vy, wz};

#if defined(USE_ROS2) && defined(USE_ROS)
    RCLCPP_INFO_THROTTLE(
        ros2_node->get_logger(), *ros2_node->get_clock(), 500 /*ms*/,
        "[OBS.CMD] [%.2f %.2f %.2f]",
        this->obs.commands[0], this->obs.commands[1], this->obs.commands[2]
    );
#endif

    // ----------------------------
    // 4) Forward + post-processing
    // ----------------------------
    this->obs.actions = this->Forward();

    this->ComputeOutput(
        this->obs.actions,
        this->output_dof_pos,
        this->output_dof_vel,
        this->output_dof_tau
    );

    if (!this->output_dof_pos.empty()) output_dof_pos_queue.push(this->output_dof_pos);
    if (!this->output_dof_vel.empty()) output_dof_vel_queue.push(this->output_dof_vel);
    if (!this->output_dof_tau.empty()) output_dof_tau_queue.push(this->output_dof_tau);

#ifdef CSV_LOGGER
    const std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
    this->CSVLogger(
        this->output_dof_tau,
        tau_est,
        this->obs.dof_pos,
        this->output_dof_pos,
        this->obs.dof_vel
    );
#endif
}


// ----------------------------
// Forward (unchanged)
// ----------------------------
std::vector<float> RL_Real_G1_EDU23::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
        return this->obs.actions;

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
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

// ----------------------------
// CRC32
// ----------------------------
uint32_t RL_Real_G1_EDU23::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}

// ----------------------------
// InitLowCmd (SAFE)
// ----------------------------
void RL_Real_G1_EDU23::InitLowCmd()
{
    auto &mc = this->unitree_low_command.motor_cmd();
    const int n = (int)mc.size();
    for (int i = 0; i < n; ++i)
    {
        mc[i].mode() = 0; // SAFE: disabled by default
        mc[i].q() = 0;
        mc[i].kp() = 0;
        mc[i].dq() = 0;
        mc[i].kd() = 0;
        mc[i].tau() = 0;
    }
}

// ----------------------------
// Plot (optional)
// ----------------------------
#ifdef PLOT
void RL_Real_G1_EDU23::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);

    plt::cla();
    plt::clf();

    // NOTE: These plots use raw motor_state index i, not policy mapping.
    // If you need correct mapping, map i->hw_idx before indexing motor_state().
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());

        this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());

        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}
#endif

// ----------------------------
// ROS cmd_vel callback
// ----------------------------
//#if !defined(USE_CMAKE) && defined(USE_ROS)
#if defined(USE_ROS)
void RL_Real_G1_EDU23::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
#if defined(USE_ROS2) && defined(USE_ROS)
    // ✅ Thread-safe store (single source of truth)
    {
        std::lock_guard<std::mutex> lk(this->cmd_vel_mtx_);
        this->cmd_vel_ = *msg; 
        this->last_cmd_vel_time_ = ros2_node->now();
    }

    // ✅ Optional: keep legacy variable in sync if other code reads it
    // (safe because it's just a copy; remove if you don't use this->cmd_vel anywhere else)
    this->cmd_vel_ = *msg;

    // ✅ Throttled log: confirms reception only
    RCLCPP_INFO_THROTTLE(
        ros2_node->get_logger(), *ros2_node->get_clock(), 200 /*ms*/,
        "[CMD_VEL RX] vx=%.3f vy=%.3f wz=%.3f",
        msg->linear.x, msg->linear.y, msg->angular.z
    );

#else  // ROS1

    // ROS1: store directly (single-threaded callback typically)
    this->cmd_vel_ = *msg;

    static int cnt = 0;
    if ((cnt++ % 10) == 0)
    {
        std::cout << "[CMD_VEL RX] vx=" << this->cmd_vel.linear.x
                  << " vy=" << this->cmd_vel.linear.y
                  << " wz=" << this->cmd_vel.angular.z << std::endl;
    }

#endif
}
#endif



// ----------------------------
// MAIN
// ----------------------------
#if defined(USE_ROS1) && defined(USE_ROS)
static void signalHandler(int)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar_g1_edu23");
    RL_Real_G1_EDU23 app(argc, argv);
    ros::spin();

#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto app = std::make_shared<RL_Real_G1_EDU23>(argc, argv);
    rclcpp::spin(app->ros2_node);
    rclcpp::shutdown();

#else
    RL_Real_G1_EDU23 app(argc, argv);
    while (1) { sleep(10); }
#endif

    return 0;
}
