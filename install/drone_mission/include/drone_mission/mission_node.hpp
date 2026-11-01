#ifndef DRONE_MISSION__MISSION_NODE_HPP_
#define DRONE_MISSION__MISSION_NODE_HPP_

#include <array>
#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class MissionNode : public rclcpp::Node
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State { Idle, Takeoff, Waypoints, Pickup, Land, Done };

  // ROS entities: subscribers receive state, publishers send commands, and the wall timer schedules control.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detections_subscriber_;
  
  geometry_msgs::msg::Pose starting_pose_;
  geometry_msgs::msg::PointStamped latest_point_{};
  nav_msgs::msg::Odometry latest_odometry_;

  bool has_odometry_{false};
  bool has_point_{false};
  State state_{State::Idle};
  std::array<double, 3> waypoint_ {0.1, 0.1, 0.1};
  int leg_{0};
  double leg_len_{0.5};
  double spacing_{0.5};
  int max_legs_{24};

  static constexpr std::array<std::array<int, 2>, 4> kDirs{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};   // E, N, W, S


  double takeoff_altitude_{2.0};
  double kp_xy_{1.0};
  double kp_z_{1.0};
  double kp_yaw_{1.0};
  double max_vel_xy_{1.0};
  double max_vel_z_{0.5};
  double position_tolerance_{0.2};
  double control_rate_hz_{20.0};

  void control_loop();
  geometry_msgs::msg::Twist step_takeoff();
  geometry_msgs::msg::Twist step_waypoints();
  geometry_msgs::msg::Twist step_land();
  geometry_msgs::msg::Twist pickup();

  std::array<double, 3> PID_controller(const std::array<double, 3> & target_pose_, const std::array<double, 3> & current_pose_) const;

  static std::array<double, 3> world_to_body(const std::array<double, 3> & v_world, double yaw);
  static double get_yaw(const geometry_msgs::msg::Quaternion & q);
  static double clamp_velocity(double v, double limit);
  bool reached(const std::array<double, 3> & target) const;
  const geometry_msgs::msg::Pose & get_pose() const; 

};

#endif  // DRONE_MISSION__MISSION_NODE_HPP_
