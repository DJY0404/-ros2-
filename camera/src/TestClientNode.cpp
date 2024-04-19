#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgspack/srv/testsrv.hpp" 

class TestClientNode : public rclcpp::Node {
public:
    TestClientNode() : Node("test_node") {
        // Create a timer to periodically call the service
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TestClientNode::call_service, this));
        // Read parameter value
        this->declare_parameter("my_parameter", "default_value");
        parameter_value_ = this->get_parameter("my_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter value: %s", parameter_value_.c_str());
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::string parameter_value_;

    void call_service() {
        // Create a client to call the service
        auto client = this->create_client<msgspack::srv::Testsrv>("my_service");

        // Wait for the service to become available
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        // Create a request to send to the service
        auto request = std::make_shared<msgspack::srv::Testsrv::Request>();
        request->t1 = 42; // Set the request value (integer)

        // Call the service
        auto future_result = client->async_send_request(request);

        // Wait for the response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future_result.get(); // Get the response from the future

            // Process the response
            RCLCPP_INFO(this->get_logger(), "Service response: %ld", response->t2);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
