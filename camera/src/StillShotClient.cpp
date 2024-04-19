#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"  // Include the Trigger service message

class StillShotClient : public rclcpp::Node {
public:
    StillShotClient() : Node("still_shot_client") {
        // Create a client to call the capture_still_shot service
        client_ = this->create_client<std_srvs::srv::Trigger>("/capture_still_shot");

        // Call the service when the node is started
        this->call_service();
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

    void call_service() {
        // Wait for the service to become available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // Send a request to the service
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);

        // Wait for the response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Still shot captured successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture still shot: %s", result->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call the service.");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StillShotClient>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
