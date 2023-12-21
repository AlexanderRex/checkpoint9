#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode()
      : Node("pre_approach"), current_yaw(0.0), integral(0.0), prev_error(0.0) {
    this->declare_parameter<float>("obstacle", 0.0);
    this->declare_parameter<float>("degrees", 0.0);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachNode::scan_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PreApproachNode::timer_callback, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    current_scan_ = *msg;
  }

  void timer_callback() {
    auto obstacle_distance = this->get_parameter("obstacle").as_double();
    auto rotation_degrees = this->get_parameter("degrees").as_double();

    // Получаем индекс переднего луча
    size_t center_index = current_scan_.ranges.size() / 2;

    // Проверяем дистанцию переднего луча
    if (current_scan_.ranges[center_index] < obstacle_distance) {
      stop_robot();
      rotate_robot(rotation_degrees);
      return;
    }

    move_forward();
  }

  void move_forward() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.5;
    publisher_->publish(msg);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    publisher_->publish(msg);
  }

  void rotate_robot(double target_degrees) {
    double target_yaw = current_yaw + (target_degrees * M_PI / 180.0);

    if (target_yaw > M_PI)
      target_yaw -= 2 * M_PI;
    else if (target_yaw < -M_PI)
      target_yaw += 2 * M_PI;

    const double kP = 0.5;
    const double kI = 0.1;
    const double kD = 0.1;

    rclcpp::Rate rate(10);
    std::chrono::steady_clock::time_point last_time =
        std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - last_time;
      last_time = now;
      double dt = elapsed.count();

      double error = target_yaw - current_yaw;
      integral += error * dt;
      double derivative = (error - prev_error) / dt;
      prev_error = error;

      double angular_z = kP * error + kI * integral + kD * derivative;
      geometry_msgs::msg::Twist twist;
      twist.angular.z = angular_z;
      publisher_->publish(twist);

      if (std::abs(error) < 0.01) {
        break;
      }

      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }

    stop_robot();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan current_scan_;
  double current_yaw;          // Текущий угол поворота
  double integral, prev_error; // Для PID-контроллера
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
