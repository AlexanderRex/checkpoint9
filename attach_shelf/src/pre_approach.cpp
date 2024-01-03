#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach"), obstacle(0.0), degrees(0.0) {
    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0.0);

    getting_params();

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachNode::scan_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  bool obstacle_detected = false;

  void getting_params() {
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<float>();
    RCLCPP_INFO(this->get_logger(), "Obstacle threshold set to: %f", obstacle);
    RCLCPP_INFO(this->get_logger(), "Rotation degrees set to: %f", degrees);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float distance_to_obstacle = msg->ranges[msg->ranges.size() / 2];

    if (distance_to_obstacle < obstacle) {
      RCLCPP_WARN(this->get_logger(),
                  "Obstacle detected! Stopping and rotating robot.");
      stop_robot();
      rotate_robot();
      obstacle_detected = true; // Установка флага
    } else if (!obstacle_detected) {
      move_forward();
    }
  }

  void odom_сallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double s = msg->pose.pose.orientation.w;
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;

    // Calculate yaw angle from the quaternion
    current_yaw = atan2(2.0 * (y * x + s * z), s * s + x * x - y * y - z * z);
  }

  void move_forward() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.25;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward");
  }

  void stop_robot() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Stopping robot");
  }

  void rotate_robot() {
    geometry_msgs::msg::Twist msg;
    msg.angular.z = 0.5;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Rotating robot");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float obstacle;
  float degrees;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
