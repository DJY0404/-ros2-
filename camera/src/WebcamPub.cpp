/*

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher() : Node("cam_pub") {
        // Create publisher for image messages
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam/camimage", 16);

        // Open the default webcam
        capture_ = cv::VideoCapture(0);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam");
            rclcpp::shutdown();
        }

        // Start publishing webcam frames
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&WebcamPublisher::publish_frames, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture capture_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_frames() {
        cv::Mat frame;
        capture_ >> frame;
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamPublisher>();
    rclcpp::spin(node); // Only call spin() once
    rclcpp::shutdown();
    return 0;
}
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher() : Node("cam_pub") {
        // Create publisher for image messages
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam/camimage", 10);

        // Open the default webcam
        capture_ = cv::VideoCapture(0);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam");
            rclcpp::shutdown();
        }

        // Start capturing frames in a separate thread
        capture_thread_ = std::thread(&WebcamPublisher::capture_frames, this);
    }

    ~WebcamPublisher() {
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture capture_;
    std::thread capture_thread_;

    void capture_frames() {
        while (rclcpp::ok()) {
            cv::Mat frame;
            capture_ >> frame;
            if (!frame.empty()) {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                publisher_->publish(*msg);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamPublisher>();
    rclcpp::spin(node); // Only call spin() once
    rclcpp::shutdown();
    return 0;
}
