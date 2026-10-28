#include "drone_mission/mission_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

MissionNode::MissionNode(const rclcpp::NodeOptions & options)
: Node("mission_node", options)
{
  // Parameters let mission values be changed from YAML without rebuilding the package.
  takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", takeoff_altitude_);
  const auto waypoint_values = declare_parameter<std::vector<double>>(
    "waypoints", std::vector<double>{});
  kp_xy_ = declare_parameter<double>("kp_xy", kp_xy_);
  kp_z_ = declare_parameter<double>("kp_z", kp_z_);
  kp_yaw_ = declare_parameter<double>("kp_yaw", kp_yaw_);
  max_vel_xy_ = declare_parameter<double>("max_vel_xy", max_vel_xy_);
  max_vel_z_ = declare_parameter<double>("max_vel_z", max_vel_z_);
  position_tolerance_ = declare_parameter<double>("position_tolerance", position_tolerance_);
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", control_rate_hz_);

  if (waypoint_values.size() % 3 != 0) {
    throw std::invalid_argument("The waypoints parameter must contain x, y, z triples");
  }
  waypoints_.reserve(waypoint_values.size() / 3);
  for (std::size_t i = 0; i < waypoint_values.size(); i += 3) {
    waypoints_.push_back({waypoint_values[i], waypoint_values[i + 1], waypoint_values[i + 2]});
  }

  // ROS topic names match the corresponding entries in config/bridge.yaml.
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    "/X3/gazebo/command/twist", rclcpp::QoS(10));
  enable_publisher_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", rclcpp::QoS(10));
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "/model/X3/odometry", rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      latest_odometry_ = *message;
      has_odometry_ = true;
    });

  if (control_rate_hz_ <= 0.0) {
    throw std::invalid_argument("control_rate_hz must be greater than zero");
  }
  const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    [this]() {control_loop();});

  // Publish a standard ROS Bool once to arm the bridged Gazebo controller.
  std_msgs::msg::Bool enable_message;
  enable_message.data = true;
  enable_publisher_->publish(enable_message);
}

void MissionNode::control_loop()
{
  /* TODO:
   * Run the state machine, call the appropriate step function, and publish its command.
   */
}

geometry_msgs::msg::Twist MissionNode::step_takeoff()
{
  /* TODO:
   * Compute the takeoff command and transition to Waypoints when the takeoff target is reached.
   */
  return geometry_msgs::msg::Twist{};
}

geometry_msgs::msg::Twist MissionNode::step_waypoints()
{
  /* TODO:
   * Track the current waypoint, advance the waypoint index, and transition when the route is complete.
   */
  return geometry_msgs::msg::Twist{};
}

geometry_msgs::msg::Twist MissionNode::step_land()
{
  /* TODO:
   * Generate the landing command and transition to Done once the landing target is reached.
   */
  return geometry_msgs::msg::Twist{};
}

std::array<double, 3> MissionNode::world_to_body(const std::array<double, 3> & v_world, double yaw)
{
  double robot_yaw = get_yaw(get_pose().orientation);
  std::array<double , 3> v_robot = 
  //TODO

}

double MissionNode::get_yaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat;
  tf2::fromMsg(q, quat);
  std::array<double, 3> rpy {};
  tf2::Matrix3x3(quat).getRPY(rpy[0], rpy[1], rpy[2]);

  return rpy[2];
}

double MissionNode::clamp_velocity(double v, double limit)
{
  return std::clamp(v, - limit, limit);
}

bool MissionNode::reached(const std::array<double, 3> & target) const
{
    const geometry_msgs::msg::Pose & current_pose = get_pose();
    double current_path = std::sqrt(std::pow((target[0] - current_pose.position.x), 2) + std::pow((target[1] - current_pose.position.y), 2) + std::pow((target[2] - current_pose.position.z), 2));
    if (current_path <= position_tolerance_){return true;}
    else {return false;}

}

const geometry_msgs::msg::Pose & MissionNode::get_pose() const {return latest_odometry_.pose.pose;} 
