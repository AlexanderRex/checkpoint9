#include "my_components/attach_client.hpp"

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  client_ =
      this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
  call_approach_shelf_service();
}

void AttachClient::call_approach_shelf_service() {
  auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto result_future = client_->async_send_request(
      request, [this](std::shared_future<
                      std::shared_ptr<attach_shelf::srv::GoToLoading::Response>>
                          response_future) {
        auto response = response_future.get();
        if (response->complete) {
          RCLCPP_INFO(this->get_logger(),
                      "Approach shelf service call successful");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Approach shelf service call failed");
        }
      });
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
