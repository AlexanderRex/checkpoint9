#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <vector>

class ShelfLegDetectionService : public rclcpp::Node {
public:
  ShelfLegDetectionService() : Node("shelf_leg_detection_service") {

    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/detect_shelf_legs",
        std::bind(&ShelfLegDetectionService::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ShelfLegDetectionService::laser_scan_callback, this,
                      std::placeholders::_1));
  }

private:
  void handle_service_request(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    // Логика обработки запроса сервиса
    if (detect_shelf_legs()) {
      response->success = true;
      response->message = "Legs detected";
    } else {
      response->success = false;
      response->message = "Only one or no shelf leg detected";
    }
  }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Обработка данных сканера
    current_scan_ = *msg;
  }

  bool detect_shelf_legs() {
    bool is_currently_high_intensity = false;

    for (const auto &intensity : current_scan_.intensities) {
      if (intensity >= intensity_threshold_) {
        if (!is_currently_high_intensity) {
          legs_detected++;
          is_currently_high_intensity = true;
        }
      } else {
        is_currently_high_intensity = false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Number of legs detected: %d",
                legs_detected);
    return legs_detected > 1;
  }

  int legs_detected;
  sensor_msgs::msg::LaserScan current_scan_;
  float intensity_threshold_ =
      8000.0; // порог интенсивности для обнаружения ножек
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShelfLegDetectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
