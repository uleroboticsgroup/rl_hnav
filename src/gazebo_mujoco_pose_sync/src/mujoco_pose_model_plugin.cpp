#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
    z_offset_ = sdf->HasElement("z_offset") ? sdf->Get<double>("z_offset") : 0.0;

    // ---- ROS2 init ----
    node_ = gazebo_ros::Node::Get(sdf);

    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_pose_ = *msg;
        have_pose_ = true;
      });

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/mujoco/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_joints_ = *msg;
        have_joints_ = true;
      });

    // We don't need a separate thread if we use gazebo_ros::Node 
    // but we need to ensure the messages are processed.
    // Actually, gazebo_ros::Node spins in the background.

    // ---- Make the model a pure kinematic puppet ----
    // Do NOT use SetKinematic(true) — it breaks FK propagation of SetPosition.
    // Instead: disable gravity + collisions, and connect to WorldUpdateEnd
    // to re-apply poses AFTER ODE has done its (now harmless) step.
    for (auto &link : model_->GetLinks()) {
      link->SetGravityMode(false);
      link->SetCollideMode("none");
      link->SetAutoDisable(false);
    }

    // Also disable self-collision on the model
    model_->SetSelfCollide(false);

    // Connect to WorldUpdateEnd so our pose writes happen AFTER ODE step,
    // ensuring we have the final word on link positions.
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
        std::bind(&MujocoPoseModelPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
      "MujocoPoseModelPlugin: puppet mode (gravity=OFF, collisions=NONE) for %zu links",
      model_->GetLinks().size());
    RCLCPP_INFO(node_->get_logger(), "MujocoPoseModelPlugin listening on '%s' (target_link='%s')", 
                pose_topic_.c_str(), target_link_.empty() ? "<model>" : target_link_.c_str());
  }

  void OnUpdate()
  {
    if (!model_) return;
    geometry_msgs::msg::PoseStamped pose_msg;
    sensor_msgs::msg::JointState joints_msg;
    bool pose_ready = false;
    bool joints_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_pose_) {
        pose_msg = last_pose_;
        pose_ready = true;
      }
      if (have_joints_) {
        joints_msg = last_joints_;
        joints_ready = true;
      }
    }

    if (pose_ready) {
      // MuJoCo publishes the PELVIS world pose (qpos[0..6]).
      // Gazebo model root is base_footprint, which is 0.92m below pelvis
      // via a fixed joint (origin xyz="0 0 0.92").
      //
      // CRITICAL: We must rotate the offset into the pelvis frame.
      // base_footprint_pos = pelvis_pos + pelvis_rot * (0, 0, -0.92)
      // If we just subtract 0.92 in world-Z, any pelvis roll/pitch
      // creates a lateral offset error → visible as over-sway.
      const auto &q = pose_msg.pose.orientation;
      ignition::math::Quaterniond quat(q.w, q.x, q.y, q.z);

      // Offset from pelvis to base_footprint in pelvis-local frame
      ignition::math::Vector3d local_offset(0, 0, -0.92);
      ignition::math::Vector3d world_offset = quat.RotateVector(local_offset);

      ignition::math::Pose3d p(
        pose_msg.pose.position.x + world_offset.X(),
        pose_msg.pose.position.y + world_offset.Y(),
        pose_msg.pose.position.z + world_offset.Z(),
        0.0, 0.0, 0.0);
      p.Rot() = quat;

      if (target_link_.empty())
      {
        model_->SetWorldPose(p);
      }
      else
      {
        auto link = model_->GetLink(target_link_);
        
        // Try to find it by suffix if it's nested
        if (!link) {
          for (auto const &l : model_->GetLinks()) {
            std::string scoped_name = l->GetScopedName();
            std::string suffix = "::" + target_link_;
            if (l->GetName() == target_link_ || (scoped_name.size() >= suffix.size() && scoped_name.compare(scoped_name.size() - suffix.size(), suffix.size(), suffix) == 0)) {
              link = l;
              break;
            }
          }
        }

        if (link) {
          link->SetWorldPose(p);
        } else {
          static bool warned = false;
          if (!warned) {
            std::string available_links = "";
            for (auto const &l : model_->GetLinks()) {
              available_links += l->GetName() + " (" + l->GetScopedName() + "), ";
            }
            RCLCPP_WARN(node_->get_logger(), "Target link '%s' not found in model '%s'. Available links: [%s]", 
                        target_link_.c_str(), model_->GetName().c_str(), available_links.c_str());
            warned = true;
          }
        }
      }
    }

    if (joints_ready) {
      size_t num_joints = std::min(joints_msg.name.size(), joints_msg.position.size());
      for (size_t i = 0; i < num_joints; ++i) {
        auto j = model_->GetJoint(joints_msg.name[i]);
        if (j) {
           j->SetPosition(0, joints_msg.position[i], true);
        }
      }
    }

    // After setting base pose + joint angles, zero all motion states
    // so ODE has no residual that can accumulate into drift.
    for (auto &link : model_->GetLinks()) {
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      link->SetForce(ignition::math::Vector3d::Zero);
      link->SetTorque(ignition::math::Vector3d::Zero);
    }
    for (auto &j : model_->GetJoints()) {
      j->SetForce(0, 0.0);
    }
  }

  ~MujocoPoseModelPlugin() override = default;

private:
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  gazebo::event::ConnectionPtr update_connection_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  sensor_msgs::msg::JointState last_joints_;
  bool have_pose_{false};
  bool have_joints_{false};

  std::string pose_topic_;
  std::string target_link_;
  double z_offset_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(MujocoPoseModelPlugin)

}  // namespace gazebo_mujoco_pose_sync
