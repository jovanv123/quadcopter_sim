  #include "drone_mission/mission_node.hpp"

  #include <chrono>
  #include <stdexcept>
  #include <string>
  #include <utility>

  MissionNode::MissionNode(const rclcpp::NodeOptions & options)
  : Node("mission_node", options)
  {
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", takeoff_altitude_);

    kp_xy_ = declare_parameter<double>("kp_xy", kp_xy_);
    kp_z_ = declare_parameter<double>("kp_z", kp_z_);
    kp_yaw_ = declare_parameter<double>("kp_yaw", kp_yaw_);
    max_vel_xy_ = declare_parameter<double>("max_vel_xy", max_vel_xy_);
    max_vel_z_ = declare_parameter<double>("max_vel_z", max_vel_z_);
    position_tolerance_ = declare_parameter<double>("position_tolerance", position_tolerance_);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", control_rate_hz_);

    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/X3/gazebo/command/twist", rclcpp::QoS(10));

    enable_publisher_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", rclcpp::QoS(10));

    right_gripper_publisher_ = create_publisher<std_msgs::msg::Float64>("/x3/gripper/right_cmd", rclcpp::QoS(10));

    left_gripper_publisher_ = create_publisher<std_msgs::msg::Float64>("/x3/gripper/left_cmd", rclcpp::QoS(10));

    can_attach_publisher_ = create_publisher<std_msgs::msg::Empty>("/x3/gripper/attach/trash_can_1", rclcpp::QoS(10));

    bottle_attach_publisher_ = create_publisher<std_msgs::msg::Empty>("/x3/gripper/attach/trash_bottle_1", rclcpp::QoS(10));

    can_detach_publisher_ = create_publisher<std_msgs::msg::Empty>("/x3/gripper/detach/trash_can_1", rclcpp::QoS(10));
    
    bottle_detach_publisher_ = create_publisher<std_msgs::msg::Empty>("/x3/gripper/detach/trash_bottle_1", rclcpp::QoS(10));

    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("/model/X3/odometry", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        has_odometry_ = true;
      });

    detections_subscriber_ = create_subscription<drone_interfaces::msg::Detection>("/detections/closest_world", rclcpp::QoS(10),
      [this](const drone_interfaces::msg::Detection::SharedPtr message) {
        if (!has_point_) {
          latest_point_ = *message;
          has_point_ = true;
        }
      });

    if (control_rate_hz_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be greater than zero");
    }
    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      [this]() {control_loop();});
  }

  void MissionNode::control_loop()
  {
    enable_publisher_->publish(std_msgs::msg::Bool().set__data(true));

    switch (state_) {
      case State::Idle: 
        if (has_odometry_ && can_detach_publisher_->get_subscription_count() > 0 && bottle_detach_publisher_->get_subscription_count() > 0) 
          {
            detach();
            starting_pose_ = get_pose(); 
            state_ = State::Takeoff;
          } break;

      case State::Takeoff: twist_publisher_->publish(step_takeoff()); break;
      case State::Waypoints: twist_publisher_->publish(step_waypoints()); break;
      case State::Approach: twist_publisher_->publish(Approach()); break;
      case State::Pickup: twist_publisher_->publish(Pickup()); break;
      case State::Land: twist_publisher_->publish(step_land()); break;
      case State::Hold: twist_publisher_->publish(Hold()); break;
    }
  }

  geometry_msgs::msg::Twist MissionNode::step_takeoff()
  {
    const std::array<double, 3> target {starting_pose_.position.x, starting_pose_.position.y, takeoff_altitude_};
    if (reached(target)) {state_ = State::Waypoints;}
    return go_to(target);
  }

  geometry_msgs::msg::Twist MissionNode::step_waypoints()
  {
    const std::array<double, 3> target {waypoint_[0], waypoint_[1], takeoff_altitude_};

    if (reached(target) || has_point_) {
      if (leg_ >= max_legs_ || has_point_) {
        leg_ = 0;
        leg_len_ = 0.5;
        state_ = has_point_ ? State::Approach : State::Land;
        return geometry_msgs::msg::Twist{};
      }
      const auto & d = kDirs[leg_ % 4];
      waypoint_[0] += d[0] * leg_len_ * spacing_;
      waypoint_[1] += d[1] * leg_len_ * spacing_;
      if (leg_ % 2 == 1) {leg_len_ += 0.5;}
      ++leg_;
    }

    return go_to(target);
  }

  geometry_msgs::msg::Twist MissionNode::Approach()
  {
    const std::array<double, 3> target {latest_point_.position.x, latest_point_.position.y, takeoff_altitude_};
    if (reached(target)) {state_ = State::Pickup;}
    return go_to(target);
  }

  geometry_msgs::msg::Twist MissionNode::Pickup()
  {
    std::array<double, 3> target {latest_point_.position.x, latest_point_.position.y, takeoff_altitude_};
    if (latest_point_.label == 1 || latest_point_.label == 2) {
      target[2] = latest_point_.position.z + 0.23;
    }
    if (reached(target)) {enter_hold();}
    return go_to(target);
  }

  geometry_msgs::msg::Twist MissionNode::step_land()
  {
    const std::array<double, 3> target {starting_pose_.position.x, starting_pose_.position.y, 0.17};
    if (reached(target)) {enter_hold();}
    return go_to(target);
  }

  geometry_msgs::msg::Twist MissionNode::Hold()
  {
    if (is_picking_) {close_gripper(); is_picking_ = false;}
    return go_to(hold_pose_);
  }

  void MissionNode::enter_hold()
  {
    hold_pose_ = current_position();
    if (state_ == State::Pickup){
      is_picking_ = true;
    }
    state_ = State::Hold;
  }

  void MissionNode::close_gripper()
  {
    left_gripper_publisher_->publish(std_msgs::msg::Float64().set__data(0.04));
    right_gripper_publisher_->publish(std_msgs::msg::Float64().set__data(0.04));
    is_picking_ = false;
  }
  void MissionNode::detach()
  {
    bottle_detach_publisher_->publish(std_msgs::msg::Empty{});
    can_detach_publisher_->publish(std_msgs::msg::Empty{});
  }


  geometry_msgs::msg::Twist MissionNode::go_to(const std::array<double, 3> & target) const
  {
    const auto v = PID_controller(target, current_position());

    geometry_msgs::msg::Twist msg;
    msg.linear.x = v[0];
    msg.linear.y = v[1];
    msg.linear.z = v[2];
    return msg;
  }

  std::array<double, 3> MissionNode::current_position() const
  {
    const auto & p = get_pose().position;
    return {p.x, p.y, p.z};
  }

  std::array<double, 3> MissionNode::world_to_body(const std::array<double, 3> & v_world, double yaw)
  {
    const Eigen::Map<const Eigen::Vector3d> v(v_world.data());
    const Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d v_body = R.transpose() * v;

    return {v_body.x(), v_body.y(), v_body.z()};
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
    return std::clamp(v, -limit, limit);
  }

  bool MissionNode::reached(const std::array<double, 3> & target) const
  {
    const auto p = current_position();
    return std::hypot(target[0] - p[0], target[1] - p[1], target[2] - p[2]) <= position_tolerance_;
  }

  std::array<double, 3> MissionNode::PID_controller(const std::array<double, 3> & target_pose_, const std::array<double, 3> & current_pose_) const
  {
    std::array<double, 3> v_world {};

    v_world[0] = clamp_velocity(kp_xy_ * (target_pose_[0] - current_pose_[0]), max_vel_xy_);
    v_world[1] = clamp_velocity(kp_xy_ * (target_pose_[1] - current_pose_[1]), max_vel_xy_);
    v_world[2] = clamp_velocity(kp_z_ * (target_pose_[2] - current_pose_[2]), max_vel_z_);

    return world_to_body(v_world, get_yaw(get_pose().orientation));
  }

  const geometry_msgs::msg::Pose & MissionNode::get_pose() const {return latest_odometry_.pose.pose;}