#include "drone_mission/tracker_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{

    detections_subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/detections/closest", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr message){
            if(!has_point_)
            {
                has_point_ = true;
                process_input_point(*message);
            }
        }
    );
    
    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/color/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
        {
            const Eigen::Matrix3d K = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(message->k.data());;
            Kinv_ = K.inverse();
            has_k_ = true;
            camera_info_subscriber_.reset();

        }
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    world_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/detections/closest_world", 10);
}

void TrackerNode::process_input_point(const geometry_msgs::msg::PointStamped & message) {
    if(!has_k_){return;}
    latest_point_ = message;
    const std::array<double, 3> p{latest_point_.point.x, latest_point_.point.y, latest_point_.point.z};
    const auto point_3d = reproject(p);
    const std::array<double, 3> chosen_point{point_3d.x(), point_3d.y(), point_3d.z()};

    geometry_msgs::msg::PointStamped p_cam;
    p_cam.header = message.header;            // frame = camera_color_optical_frame, stamp = image time
    p_cam.point.x = chosen_point[0];
    p_cam.point.y = chosen_point[1];
    p_cam.point.z = chosen_point[2];

    try {
        const auto p_world = tf_buffer_->transform(p_cam, world_frame_, tf2::durationFromSec(0.1));
        world_pub_->publish(p_world);
    } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed: %s", e.what());
    }

}

Eigen::Vector3d TrackerNode::reproject(const std::array<double, 3>& point) const
{
    const Eigen::Vector3d output = point[2] * (Kinv_ * Eigen::Vector3d(point[0], point[1], 1.0));
    return output;
}

