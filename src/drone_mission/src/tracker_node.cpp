#include "drone_mission/tracker_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{
  detections_subscriber_ = create_subscription<drone_interfaces::msg::Detection>(
    "/detections/closest", rclcpp::QoS(10),
    [this](const drone_interfaces::msg::Detection::SharedPtr message) {
      if (!has_point_) {
        process_input_point(*message);
      }
    });

  camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/color/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      const Eigen::Matrix3d K =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(message->k.data());
      Kinv_ = K.inverse();
      has_k_ = true;
      camera_info_subscriber_.reset();
    });

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  world_pub_ = create_publisher<drone_interfaces::msg::Detection>("/detections/closest_world", 10);
}

void TrackerNode::process_input_point(const drone_interfaces::msg::Detection & message)
{
  if (!has_k_) {return;}
  latest_point_ = message;
  const std::array<double, 3> p{message.position.x, message.position.y, message.position.z};
  const Eigen::Vector3d point_3d = reproject(p);

  geometry_msgs::msg::PointStamped p_cam;
  p_cam.header = message.header;
  p_cam.point.x = point_3d.x();
  p_cam.point.y = point_3d.y();
  p_cam.point.z = point_3d.z();

  try {
    const auto p_world = tf_buffer_->transform(p_cam, world_frame_, tf2::durationFromSec(0.1));

    drone_interfaces::msg::Detection out;
    out.header = p_world.header;
    out.label = message.label;
    out.position = p_world.point;
    world_pub_->publish(out);
    has_point_ = true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed: %s", e.what());
  }
}

Eigen::Vector3d TrackerNode::reproject(const std::array<double, 3> & point) const
{
  return point[2] * (Kinv_ * Eigen::Vector3d(point[0], point[1], 1.0));
}