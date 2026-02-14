/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 *
 * rl_mujoco: MuJoCo GUI + ROS2 cmd_vel navigation mode
 */

#ifndef RL_MUJOCO_HPP
#define RL_MUJOCO_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <mutex>
#include <atomic>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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

class RL_Mujoco : public RL
{
public:
    RL_Mujoco(int argc, char **argv);
    ~RL_Mujoco();

    // MuJoCo UI
    std::unique_ptr<mj::Simulate> sim;
    static RL_Mujoco* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;

    void RunModel();
    void RobotControl();

    // loops
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_ros_spin;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data = nullptr;
    mjModel *mj_model = nullptr;
    std::string scene_name;

    // ROS2 interface
    std::shared_ptr<rclcpp::Node> ros2_node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;

    // cmd_vel storage (thread-safe)
    std::mutex cmd_vel_mtx_;
    geometry_msgs::msg::Twist cmd_vel_;
    std::atomic<bool> have_cmd_vel_{false};

    // optional: param node client (comme rl_sim ROS2)
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client;

    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // joystick (linux)
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;
    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();

    // helpers
    void SpinRosOnce_();

    // params
    bool navigation_mode_param_ = true;
    double cmd_vel_timeout_sec_ = 0.6;
    std::string cmd_vel_topic_ = "/cmd_vel";

    // timeout bookkeeping
    rclcpp::Time last_cmd_vel_stamp_{0, 0, RCL_ROS_TIME};

};

#endif // RL_MUJOCO_HPP
