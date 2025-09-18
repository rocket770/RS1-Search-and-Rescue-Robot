#pragma once
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_cpp_v3/action_node.h>

namespace nav2_bt {

class RandomGoal : public BT::SyncActionNode
{
public:
  // ctor used by BT when creating the node
  RandomGoal(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::SyncActionNode(name, conf),
    clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(clock_)),
    tf_listener_(*tf_buffer_)
  {}

  // trivial default ctor to satisfy pluginlib (not used by BT execution)
  RandomGoal()
  : BT::SyncActionNode("RandomGoal", BT::NodeConfiguration{}),
    clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(clock_)),
    tf_listener_(*tf_buffer_)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Output PoseStamped goal"),
      BT::InputPort<std::string>(
        "frame_id", std::string("map"), "TF frame to place goal in"),
      BT::InputPort<double>(
        "radius", 5.0, "Random radius (m) around current pose")
    };
  }

  BT::NodeStatus tick() override
  {
    // Inputs
    std::string frame_id = "map";
    double radius = 5.0;
    (void)getInput("frame_id", frame_id);
    (void)getInput("radius", radius);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(rclcpp::get_logger("RandomGoal"), "TF unavailable: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // Random point in circle
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const double r = radius * std::sqrt(uni(rng));
    const double theta = 2.0 * M_PI * uni(rng);
    const double dx = r * std::cos(theta);
    const double dy = r * std::sin(theta);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = clock_->now();
    goal.header.frame_id = frame_id;
    goal.pose.position.x = tf.transform.translation.x + dx;
    goal.pose.position.y = tf.transform.translation.y + dy;
    goal.pose.position.z = 0.0;

    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    goal.pose.orientation = tf2::toMsg(q);

    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace nav2_bt
