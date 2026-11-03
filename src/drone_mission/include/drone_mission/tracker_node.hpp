#ifndef DRONE_MISSION__TRACKER_NODE_HPP_
#define DRONE_MISSION__TRACKER_NODE_HPP_

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "drone_interfaces/msg/detection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"


class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  void process_input_point(const drone_interfaces::msg::Detection & message);
  Eigen::Vector3d reproject(const std::array<double, 3> & point) const;   // pixel (u, v) + depth -> camera frame

  rclcpp::Subscription<drone_interfaces::msg::Detection>::SharedPtr detections_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

  rclcpp::Publisher<drone_interfaces::msg::Detection>::SharedPtr world_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string world_frame_{"x3/odom"}; 

  drone_interfaces::msg::Detection latest_point_;
  Eigen::Matrix3d Kinv_{Eigen::Matrix3d::Identity()};
  bool has_point_{false};
  bool has_k_{false};
};

#endif  // DRONE_MISSION__TRACKER_NODE_HPP_