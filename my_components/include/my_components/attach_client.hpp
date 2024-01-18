#ifndef ATTACH_CLIENT_HPP_
#define ATTACH_CLIENT_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  void call_approach_shelf_service();

  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
};

} // namespace my_components

#endif // ATTACH_CLIENT_HPP_
