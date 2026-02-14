#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex>
#include <string>
#include <thread>

namespace gazebo_mujoco_pose_sync
{
class MujocoPoseModelPlugin : public gazebo::ModelPlugin
{
public:
  MujocoPoseModelPlugin() = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    // ---- params from SDF ----
    pose_topic_ = sdf->HasElement("pose_topic") ? sdf->Get<std::string>("pose_topic") : "/mujoco/base_pose";
    target_link_ = sdf->HasElement("target_link") ? sdf->Get<std::string>("target_link") : ""; // empty => whole model

    // ---- ROS2 init ----
    // rclcpp::is_initialized() is not available in Humble.
    // Safe approach: try init and ignore if already initialized.
    try {
      if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
      }
    } catch (...) {
      // If already initialized, ignore
    }

    node_ = std::make_shared<rclcpp::Node>("gazebo_mujoco_pose_sync");

    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_pose_ = *msg;
        have_pose_ = true;
      });

    // Spin in background
    spin_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MujocoPoseModelPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
                "MujocoPoseModelPlugin listening on '%s' (target_link='%s')",
                pose_topic_.c_str(),
                target_link_.empty() ? "<model>" : target_link_.c_str());
  }

  void OnUpdate()
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_pose_) return;
      pose_msg = last_pose_;
    }

    ignition::math::Pose3d p(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z,
      0.0, 0.0, 0.0);

    const auto &q = pose_msg.pose.orientation;
    ignition::math::Quaterniond quat(q.w, q.x, q.y, q.z);
    p.Rot() = quat;

    if (target_link_.empty())
    {
      model_->SetWorldPose(p);
    }
    else
    {
      auto link = model_->GetLink(target_link_);
      if (link) {
        link->SetWorldPose(p);
      }
    }
  }

  ~MujocoPoseModelPlugin() override
  {
    // Stop spinning
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "Shutting down MujocoPoseModelPlugin");
    }

    // If node is spinning, request shutdown so spin exits
    try {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    } catch (...) {}

    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_conn_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::thread spin_thread_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool have_pose_{false};

  std::string pose_topic_;
  std::string target_link_;
};

GZ_REGISTER_MODEL_PLUGIN(MujocoPoseModelPlugin)

}  // namespace gazebo_mujoco_pose_sync
