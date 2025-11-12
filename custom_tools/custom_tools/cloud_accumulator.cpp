#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::chrono_literals;

class CloudAccumulator : public rclcpp::Node
{
public:
  CloudAccumulator()
  : Node("cloud_accumulator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/accumulated_cloud");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 0.5);   
    stride_ = this->declare_parameter<int>("stride", 4); // we basically llike skip over every 'stride' points to avoid lag
    tf_cache_sec_ = this->declare_parameter<double>("tf_cache_sec", 0.2);   // amount of time we use a TF message for


    // sensor data qos for high rate reading, can likely be dropped down in future since publihs slow
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudAccumulator::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    // publish new UI periodcally so we don't overload rviz and cause more lag
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&CloudAccumulator::publishCloud, this));

    RCLCPP_INFO(
      this->get_logger(),
      "cloud_accumulator: sub=%s pub=%s target_frame=%s stride=%d publish_rate=%.2f",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      stride_, publish_rate_);
  }

private:
  // params
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double publish_rate_;
  int stride_;
  double tf_cache_sec_;

  // master cloud
  std::vector<uint8_t> master_data_;
  sensor_msgs::msg::PointCloud2 master_cloud_;
  bool has_master_ = false;

  // TF reuse
  geometry_msgs::msg::TransformStamped last_tf_;
  rclcpp::Time last_tf_time_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time now = this->now();

    // found this idea online, but its a simple TF cache, avoiding a lookup every frame
    bool need_new_tf = true;
    if (last_tf_time_.nanoseconds() > 0) {
      if ((now - last_tf_time_).seconds() < tf_cache_sec_) {
        need_new_tf = false;
        transform = last_tf_;
      }
    }

    // get the new transform
    if (need_new_tf) {
      try {
        transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.2));
        last_tf_ = transform;
        last_tf_time_ = now;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF lookup failed %s -> %s: %s",
          msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }


    sensor_msgs::msg::PointCloud2 cloud_in_target;
    // convert from camera frame to world frame
    try {
      tf2::doTransform(*msg, cloud_in_target, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "doTransform failed: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 downsampled;
    downsampleByStride(cloud_in_target, downsampled, stride_);

    // add new points to the final list, and publish it using timer
    appendToMaster(downsampled);
  }

  void downsampleByStride(
    const sensor_msgs::msg::PointCloud2 & in,
    sensor_msgs::msg::PointCloud2 & out,
    int stride)
  {
    if (stride <= 1) {
      out = in; 
      return;
    }

    // fill metadata
    out.header = in.header;
    out.height = 1;
    out.is_dense = false; // may include nans
    out.fields = in.fields;
    out.point_step = in.point_step;
    out.is_bigendian = in.is_bigendian;

    const size_t in_points = static_cast<size_t>(in.width) * static_cast<size_t>(in.height);
    const size_t in_step = in.point_step;

    // try reduce reallocations to make it faster
    out.data.reserve(in.data.size() / stride);

    size_t kept = 0;
    // skip through points to reduce the size of the accumulated points 
    for (size_t i = 0; i < in_points; i += stride) {
      const uint8_t * src = &in.data[i * in_step];
      out.data.insert(out.data.end(), src, src + in_step);
      kept++;
    }

    out.width = static_cast<uint32_t>(kept);
    out.row_step = out.point_step * out.width;
  }

  void appendToMaster(const sensor_msgs::msg::PointCloud2 & in)
  {
    // first cloud becomes the basem and add on everything later
    if (!has_master_) {
      master_cloud_ = in;
      master_data_ = in.data;
      has_master_ = true;
    } else {
      master_data_.insert(master_data_.end(), in.data.begin(), in.data.end());
      master_cloud_.data = master_data_;
      master_cloud_.width = master_cloud_.width + in.width;
      master_cloud_.row_step = master_cloud_.width * master_cloud_.point_step;
      master_cloud_.header.stamp = in.header.stamp;
    }
  }

  void publishCloud()
  {
    if (!has_master_) {
      return;
    }
    auto out = master_cloud_;
    out.header.stamp = this->now();
    out.header.frame_id = target_frame_;
    cloud_pub_->publish(out);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudAccumulator>());
  rclcpp::shutdown();
  return 0;
}
