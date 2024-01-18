#ifndef ATTACH_SERVER_HPP_
#define ATTACH_SERVER_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  void handle_service_request(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

  void calculate_and_publish_velocity();
  void publish_velocity(double linear, double angular);
  bool detect_and_publish_shelf_legs();
  void publish_cart_frame_transform(float x, float y);
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  sensor_msgs::msg::LaserScan current_scan_;
  float intensity_threshold_ = 8000.0;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  bool legs_detected_ = false;
  double min_stop_distance_ = 0.225f;
  const double max_linear_velocity_ = 0.25;
  const double max_angular_velocity_ = 1.0;
  bool should_move_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  double prev_yaw_error = 0.0;
  double integral_yaw_error = 0.0;
  bool target_reached = false;
  rclcpp::Time start_moving_time_;
  const double extra_movement_duration_ = 1.2;
  bool movement_stopped_ = false;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
  float x_center;
  float y_center;
};

} // namespace my_components

#endif // ATTACH_SERVER_HPP_
