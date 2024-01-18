#include "my_components/attach_server.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("approach_shelf", options) {

  service_ = this->create_service<attach_shelf::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(&AttachServer::handle_service_request, this,
                std::placeholders::_1, std::placeholders::_2));

  laser_scan_subscriber_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10,
          std::bind(&AttachServer::laser_scan_callback, this,
                    std::placeholders::_1));

  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AttachServer::calculate_and_publish_velocity, this));

  elevator_up_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
}

void AttachServer::handle_service_request(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

  if (!legs_detected_) {
    response->complete = false;
    return;
  }

  if (request->attach_to_shelf) {
    should_move_ = true;
    calculate_and_publish_velocity();
    response->complete = true;
  }

  if (!request->attach_to_shelf) {

    response->complete = true;
  }
}

void AttachServer::calculate_and_publish_velocity() {
  if (!should_move_)
    return;

  if (target_reached) {
    if (start_moving_time_.seconds() == 0) {
      start_moving_time_ = this->get_clock()->now();
      publish_velocity(max_linear_velocity_, 0.0);
    } else if ((this->get_clock()->now() - start_moving_time_).seconds() <
               extra_movement_duration_) {
      publish_velocity(max_linear_velocity_, 0.0);
    } else if (!movement_stopped_) {
      publish_velocity(0.0, 0.0);
      movement_stopped_ = true;
      std_msgs::msg::String msg;
      elevator_up_publisher_->publish(msg);
    }
    return;
  }

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
        "robot_base_link", "cart_frame", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  double error_distance =
      std::sqrt(std::pow(transformStamped.transform.translation.x, 2) +
                std::pow(transformStamped.transform.translation.y, 2) +
                std::pow(transformStamped.transform.translation.z, 2));

  if (error_distance <= min_stop_distance_) {
    publish_velocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Lidar in the center of shelf legs");
    target_reached = true;
    return;
  }

  tf2::Quaternion q(transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double kp_yaw = 100.0;
  double ki_yaw = 0.0;
  double kd_yaw = 0.0;

  double yaw_error = yaw;
  integral_yaw_error += yaw_error;
  double derivative_yaw_error = yaw_error - prev_yaw_error;

  double angular_velocity = kp_yaw * yaw_error + ki_yaw * integral_yaw_error +
                            kd_yaw * derivative_yaw_error;
  angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_,
                                max_angular_velocity_);

  double kp_distance = 1.0;
  double linear_velocity = kp_distance * error_distance;
  linear_velocity = std::min(linear_velocity, max_linear_velocity_);

  publish_velocity(linear_velocity, angular_velocity);

  prev_yaw_error = yaw_error;

  RCLCPP_INFO(this->get_logger(),
              "Published Twist with linear.x: %f, angular.z: %f",
              linear_velocity, angular_velocity);
}

void AttachServer::publish_cart_frame_transform(float x, float y) {
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = "robot_front_laser_base_link";
  transformStamped.child_frame_id = "cart_frame";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transformStamped);
}

void AttachServer::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  current_scan_ = *msg;
  detect_and_publish_shelf_legs();
  publish_cart_frame_transform(x_center, y_center);
}

bool AttachServer::detect_and_publish_shelf_legs() {
  int leg_count = 0;
  std::vector<std::pair<float, float>> leg_positions;
  bool is_currently_high_intensity = false;
  float current_angle = current_scan_.angle_min;

  for (size_t i = 0; i < current_scan_.intensities.size(); i++) {
    if (current_scan_.intensities[i] >= intensity_threshold_) {
      if (!is_currently_high_intensity) {
        float x = current_scan_.ranges[i] * std::cos(current_angle);
        float y = current_scan_.ranges[i] * std::sin(current_angle);
        leg_positions.push_back(std::make_pair(x, y));
        leg_count++;
        is_currently_high_intensity = true;
      }
    } else {
      is_currently_high_intensity = false;
    }
    current_angle += current_scan_.angle_increment;
  }

  if (leg_count == 2) {
    x_center = (leg_positions[0].first + leg_positions[1].first) / 2.0;
    y_center = (leg_positions[0].second + leg_positions[1].second) / 2.0;
  }

  legs_detected_ = (leg_count == 2);

  return legs_detected_;
}

void AttachServer::publish_velocity(double linear, double angular) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  vel_publisher_->publish(twist);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)