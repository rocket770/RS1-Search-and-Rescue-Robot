#pragma once
#include <random>
#include <string>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav2_bt {

class RandomGoal : public BT::SyncActionNode {
public:
  RandomGoal(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::SyncActionNode(name, conf),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared())),
    tf_listener_(*tf_buffer_)
  {}

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("frame_id", std::string("map")),
      BT::InputPort<double>("radius", 5.0)
    };
  }

  BT::NodeStatus tick() override {
    // Inputs
    std::string frame_id;
    double radius = 5.0;
    getInput("frame_id", frame_id);
    getInput("radius", radius);

    // Current pose (map -> base_link)
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(rclcpp::get_logger("RandomGoal"), "TF unavailable: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // Uniform random point in a circle of 'radius'
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    double r = radius * std::sqrt(uni(rng));
    double theta = 2.0 * M_PI * uni(rng);

    double dx = r * std::cos(theta);
    double dy = r * std::sin(theta);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = rclcpp::Clock().now();
    goal.header.frame_id = frame_id;
    goal.pose.position.x = tf.transform.translation.x + dx;
    goal.pose.position.y = tf.transform.translation.y + dy;
    goal.pose.position.z = 0.0;

    // face forward along travel direction (optional)
    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    goal.pose.orientation = tf2::toMsg(q);

    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace nav2_bt
