#ifndef RL_MUJOCO_HPP
#define RL_MUJOCO_HPP

// =====================================================
// Core RL
// =====================================================
#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

// =====================================================
// STL
// =====================================================
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

// =====================================================
// MuJoCo
// =====================================================
#include <mujoco/mujoco.h>
#include "mujoco_utils.hpp"
#include "joystick.hh"

// =====================================================
// ROS 2
// =====================================================
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

// =====================================================
// Small helper (identique à rl_sim_mujoco)
// =====================================================
class Button
{
public:
    void update(bool state)
    {
        on_press   = state && !pressed;
        on_release = !state && pressed;
        pressed    = state;
    }

    bool pressed    = false;
    bool on_press   = false;
    bool on_release = false;
};

// =====================================================
// RL_Mujoco
// =====================================================
class RL_Mujoco : public RL
{
public:
    RL_Mujoco(int argc, char **argv);
    ~RL_Mujoco();

    static RL_Mujoco* instance;

    // Ctrl+C safe exit
    void RequestExit_();

private:
    // =====================================================
    // RL overrides
    // =====================================================
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;

    void RobotControl();
    void RunModel();

    // =====================================================
    // MuJoCo
    // =====================================================
    mjModel *mj_model = nullptr;
    mjData  *mj_data  = nullptr;
    std::unique_ptr<mj::Simulate> sim;

    std::string robot_name_;
    std::string scene_name;

    // =====================================================
    // ROS2
    // =====================================================
    std::shared_ptr<rclcpp::Node> ros2_node;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber, cmd_vel_subscriber__;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    std::mutex exec_mtx_;


    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void SpinRosOnce_();

    std::mutex cmd_vel_mtx_;
    geometry_msgs::msg::Twist cmd_vel_;
    std::atomic<bool> have_cmd_vel_{false};
    rclcpp::Time last_cmd_vel_stamp_;

    bool navigation_mode_param_ = true;
    double cmd_vel_timeout_sec_ = 0.6;
    std::string cmd_vel_topic_ = "/cmd_vel";

    // --- base pose publishing (bridge MuJoCo → Gazebo/Nav2)
    bool publish_base_pose_ = true;
    double publish_pose_hz_ = 30.0;
    rclcpp::Time last_pose_pub_stamp_;

    void PublishBasePose_();

    // =====================================================
    // Boot / reset helpers
    // =====================================================
    bool force_stand_keyframe_ = true;
    std::string stand_key_name_ = "stand";

    void ApplyStandKeyframeIfAvailable_();

    // =====================================================
    // Loops
    // =====================================================
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_ros_spin;
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;

    // =====================================================
    // Joystick (UNE SEULE FOIS)
    // =====================================================
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};

    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));

    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();
};

#endif // RL_MUJOCO_HPP
