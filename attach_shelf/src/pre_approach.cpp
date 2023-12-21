#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach"), obstacle(0.0), degrees(0.0) {
    this->declare_parameter("obstacle",
                            0.0); // Устанавливаем значение по умолчанию
    this->declare_parameter("degrees",
                            0.0); // Устанавливаем значение по умолчанию

    getting_params();

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachNode::scan_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  void getting_params() {
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<float>();
    RCLCPP_INFO(this->get_logger(), "Obstacle threshold set to: %f", obstacle);
    RCLCPP_INFO(this->get_logger(), "Rotation degrees set to: %f", degrees);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float distance_to_obstacle = msg->ranges[msg->ranges.size() / 2];
    RCLCPP_INFO(this->get_logger(), "Distance to obstacle: %f",
                distance_to_obstacle);

    if (distance_to_obstacle < obstacle) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping robot.");
      stop_robot();
    } else {
      move_forward();
    }
  }

  void move_forward() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.25; // Задаем скорость движения вперед
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward");
  }

  void stop_robot() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0; // Останавливаем робота
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Stopping robot");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float obstacle; // Дистанция до препятствия
  float degrees; // Градусы для поворота (не используется в текущей логике)
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
