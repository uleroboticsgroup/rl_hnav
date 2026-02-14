/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_G1_EDU23_HPP
#define RL_REAL_G1_EDU23_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_g1.hpp"
#include "g1_edu23_joints.hpp"


#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <array>
#include <csignal>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <atomic>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

// Unitree HG topics (bare DDS)
static const std::string HG_CMD_TOPIC   = "rt/lowcmd";
static const std::string HG_IMU_TORSO   = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

// ----------------------------
// Gamepad raw mapping (unchanged)
// ----------------------------
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct
{
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;

    uint8_t idle[16];
} xRockerBtnDataStruct;

typedef union
{
    xRockerBtnDataStruct RF_RX;
    uint8_t buff[40];
} REMOTE_DATA_RX;

class Button
{
public:
    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class Gamepad
{
public:
    void update(xRockerBtnDataStruct &key_data)
    {
        lx = lx * (1 - smooth) + (std::fabs(key_data.lx) < dead_zone ? 0.0f : key_data.lx) * smooth;
        rx = rx * (1 - smooth) + (std::fabs(key_data.rx) < dead_zone ? 0.0f : key_data.rx) * smooth;
        ry = ry * (1 - smooth) + (std::fabs(key_data.ry) < dead_zone ? 0.0f : key_data.ry) * smooth;
        l2 = l2 * (1 - smooth) + (std::fabs(key_data.L2) < dead_zone ? 0.0f : key_data.L2) * smooth;
        ly = ly * (1 - smooth) + (std::fabs(key_data.ly) < dead_zone ? 0.0f : key_data.ly) * smooth;

        R1.update(key_data.btn.components.R1);
        L1.update(key_data.btn.components.L1);
        start.update(key_data.btn.components.start);
        select.update(key_data.btn.components.select);
        R2.update(key_data.btn.components.R2);
        L2.update(key_data.btn.components.L2);
        F1.update(key_data.btn.components.F1);
        F2.update(key_data.btn.components.F2);
        A.update(key_data.btn.components.A);
        B.update(key_data.btn.components.B);
        X.update(key_data.btn.components.X);
        Y.update(key_data.btn.components.Y);
        up.update(key_data.btn.components.up);
        right.update(key_data.btn.components.right);
        down.update(key_data.btn.components.down);
        left.update(key_data.btn.components.left);
    }

    float lx = 0.f;
    float rx = 0.f;
    float ry = 0.f;
    float l2 = 0.f;
    float ly = 0.f;

    float smooth = 0.03f;
    float dead_zone = 0.01f;

    Button R1, L1, start, select, R2, L2, F1, F2, A, B, X, Y, up, right, down, left;
};

enum class Mode
{
    PR = 0, // Series Control for Pitch/Roll joints
    AB = 1  // Parallel Control for A/B joints
};

enum G1JointIndex {
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,
    WaistYaw = 12,
    WaistRoll = 13,        // INVALID on EDU23 (waist locked)
    WaistA = 13,
    WaistPitch = 14,       // INVALID on EDU23 (waist locked)
    WaistB = 14,
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,   // INVALID on EDU23
    LeftWristYaw = 21,     // INVALID on EDU23
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,  // INVALID on EDU23
    RightWristYaw = 28     // INVALID on EDU23
};


// ----------------------------
// EDU+ 23DoF joint activation mask
// We keep policy-space indices [0..28] (29-DoF) but only 23 exist.
// Missing: 13,14 (waist locked), 20,21 (L wrist pitch/yaw), 27,28 (R wrist pitch/yaw)
// ----------------------------
//static inline bool is_edu23_active_joint(int policy_joint_index)
//{
//    switch (policy_joint_index)
//    {
//    case 0: case 1: case 2: case 3: case 4: case 5:
//    case 6: case 7: case 8: case 9: case 10: case 11:
//    case 12:
//    case 15: case 16: case 17: case 18: case 19:
//    case 22: case 23: case 24: case 25: case 26:
//        return true;
//    default:
//        return false;
//    }
//}



class RL_Real_G1_EDU23 : public RL
{
public:
    RL_Real_G1_EDU23(int argc, char **argv);
    ~RL_Real_G1_EDU23();

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:

    // ================================
    // Hardware execution mode
    // ================================
    enum class HwMode : uint8_t
    {
        REAL = 0,          // real robot (LowCmd publish)
        FAKE_LOWSTATE = 1, // fake LowState (pipeline test)
        CMD_VEL_ONLY = 2,  // cmd_vel listening only
        DRY_RUN = 3        // compute everything but send nothing
    };

    // ================================
    // Hardware mode (runtime)
    // ================================
    HwMode hw_mode_{HwMode::REAL};
    bool publish_lowcmd_{true};
    bool fake_rl_full_{false};   // false = mapping_only, true = rl_full
    bool cmd_vel_only_{false};


    /* ===============================
     * Fake LowState storage (runtime)
     * =============================== */
    std::vector<float> fake_hw_q_;
    std::vector<float> fake_hw_dq_;
    std::vector<float> fake_hw_tau_;
    bool fake_inited_{false};

    // RL interface overrides
    std::atomic_bool got_lowstate{false};
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void GetState_real(RobotState<float> *state);
    void GetState_test(RobotState<float> *state);
    void GetState_minimal(RobotState<float> *state);
    
    void SetCommand(const RobotCommand<float> *command) override;

    // ================================
    // Fake hardware buffers (runtime)
    // ================================
    std::vector<float> fake_hw_q;
    std::vector<float> fake_hw_dq;
    std::vector<float> fake_hw_tau;
    bool fake_inited = false;


    // loops
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
#ifdef PLOT
    std::shared_ptr<LoopFunc> loop_plot;
#endif

    // main steps
    void RunModel();
    void RobotControl();

#ifdef PLOT
    void Plot();
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
#endif

    // Unitree interface
    void InitLowCmd();
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void LowStateHandler(const void *message);
    void ImuTorsoHandler(const void *message);

    unitree::robot::b2::MotionSwitcherClient msc;
    LowCmd_   unitree_low_command;
    LowState_ unitree_low_state;
    IMUState_ unitree_imu_torso;

    Mode mode_pr;
    uint8_t mode_machine;

    Gamepad gamepad;
    REMOTE_DATA_RX remote_data_rx;

    ChannelPublisherPtr<LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<IMUState_> imutorso_subscriber;

// ===============================
// cmd_vel storage + subscription
// ===============================
#if defined(USE_ROS1) && defined(USE_ROS)

    // ROS1: single variable (no mutex needed usually)
    geometry_msgs::Twist cmd_vel_;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);

#elif defined(USE_ROS2) && defined(USE_ROS)

    // ROS2: single variable + mutex (callback thread vs control loop)
    geometry_msgs::msg::Twist cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // cmd_vel freshness tracking (ROS2 only)
    // ✅ init “never received” by using nanoseconds==0 check
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_SYSTEM_TIME};
    double cmd_vel_timeout_sec_{0.6};
    std::string cmd_vel_topic_{"/cmd_vel"};

    // thread safety for cmd_vel_ + last_cmd_vel_time_
    mutable std::mutex cmd_vel_mtx_;

    // Navigation mode control
    bool navigation_mode_param_{false};  // param initial
    bool navigation_mode_force_{false};  // force ON

    // Helper: returns true only if cmd_vel is fresh
    bool GetFreshCmdVel(float &vx, float &vy, float &wz) const;

#endif



    /* ================================
     * Base hardware YAML snapshot
     * Used to restore real-robot params
     * after policy YAML is loaded
     * ================================ */
    struct BaseYamlSnapshot
    {
        float dt = 0.005f;
        int decimation = 4;
        int num_of_dofs = 29;
        std::vector<int> joint_mapping;
        std::vector<float> default_dof_pos;
        std::vector<float> torque_limits;
        std::vector<std::string> joint_names;
        std::vector<std::string> joint_controller_names;
    };

    BaseYamlSnapshot base_snap_;

};

#endif // RL_REAL_G1_EDU23_HPP
