#ifndef DRONE_MISSION__MISSION_NODE_HPP_
#define DRONE_MISSION__MISSION_NODE_HPP_

#include <array>
#include <vector>
#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class MissionNode : public rclcpp::Node
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State { Idle, Takeoff, Waypoints, Land, Done };

  // ROS entities: subscribers receive state, publishers send commands, and the wall timer schedules control.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry latest_odometry_;

  bool has_odometry_{false};
  State state_{State::Idle};
  std::size_t current_waypoint_index_{0};
  std::vector<std::array<double, 3>> waypoints_;

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
  std::array<double, 3> world_to_body(const std::array<double, 3> & v_world, double yaw);
  double get_yaw(const geometry_msgs::msg::Quaternion & q);
  double clamp_velocity(double v, double limit);
  bool reached(const std::array<double, 3> & target) const;
  const geometry_msgs::msg::Pose & get_pose() const; 

};

#endif  // DRONE_MISSION__MISSION_NODE_HPP_
