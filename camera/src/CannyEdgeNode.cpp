#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CannyEdgeNode : public rclcpp::Node {
public:
    CannyEdgeNode() : Node("canny_edge_node") {
        // Create a subscriber for the webcam image topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "webcam/camimage", 10, std::bind(&CannyEdgeNode::image_callback, this, std::placeholders::_1));

        // Create a publisher for the processed image topic
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

        this->declare_parameter("low_threshold", 50);
        this->declare_parameter("high_threshold", 150);

        low_threshold_ = this->get_parameter("low_threshold").as_int();
        high_threshold_ = this->get_parameter("high_threshold").as_int();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    int low_threshold_;
    int high_threshold_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image message to OpenCV image format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Apply Canny edge detection filter
        cv::Mat edges;
        cv::Canny(cv_ptr->image, edges, low_threshold_, high_threshold_);

        // Convert OpenCV image format to ROS image message
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        out_msg.image = edges;

        // Publish the processed image
        pub_->publish(std::move(*out_msg.toImageMsg()));
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CannyEdgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
