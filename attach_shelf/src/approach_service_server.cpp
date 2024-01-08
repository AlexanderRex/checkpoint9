#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
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

class ShelfLegDetectionService : public rclcpp::Node {
public:
  ShelfLegDetectionService() : Node("shelf_leg_detection_service") {

    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/detect_shelf_legs",
        std::bind(&ShelfLegDetectionService::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ShelfLegDetectionService::laser_scan_callback, this,
                      std::placeholders::_1));

    vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ShelfLegDetectionService::calculate_and_publish_velocity,
                  this));

    elevator_up_publisher_ =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
  }

private:
  void handle_service_request(
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

  void calculate_and_publish_velocity() {
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
        std_msgs::msg::Empty msg;
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

    double kp_yaw = 1500.0;
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

  void publish_cart_frame_transform(float x, float y) {
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

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    current_scan_ = *msg;
    detect_and_publish_shelf_legs();
    publish_cart_frame_transform(x_center, y_center);
  }

  bool detect_and_publish_shelf_legs() {
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

  void publish_velocity(double linear, double angular) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    vel_publisher_->publish(twist);
  }

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
  double min_stop_distance_ = 0.2f;
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

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;
  float x_center;
  float y_center;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShelfLegDetectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
