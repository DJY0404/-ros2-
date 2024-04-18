#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "std_srvs/srv/trigger.hpp"


class ImageServiceServer : public rclcpp::Node {
public:
    ImageServiceServer() : Node("image_service_server") {
        // Create a subscription to the webcam image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "webcam/camimage", 10, std::bind(&ImageServiceServer::image_callback, this, std::placeholders::_1));

       // Create the capture_still_shot service
        capture_service_ = this->create_service<std_srvs::srv::Trigger>(
            "capture_still_shot", std::bind(&ImageServiceServer::capture_still_shot, this, std::placeholders::_1,std::placeholders::_2));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image message to OpenCV Mat
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

            // Process the received image (e.g., display it)
            cv::imshow("Received Image", cv_ptr->image);
            latest_image_msg_ = msg;
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }


    void capture_still_shot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!request){
            RCLCPP_ERROR(this->get_logger(), "No request");

        }
        // Check if there is a latest image available
        if (!latest_image_msg_) {
            RCLCPP_ERROR(this->get_logger(), "No image available for capture");
            response->success = false;
            response->message = "No image available for capture";
            return;
        }

        try {
            // Convert ROS Image message to OpenCV Mat
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(latest_image_msg_);

            // Define the path to save the still shot
            std::string save_path = "../capture/still_shot.jpg";  // Adjust the file path as needed

            // Save the captured frame as an image file
            if (!cv::imwrite(save_path, cv_ptr->image)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save still shot");
                response->success = false;
                response->message = "Failed to save still shot";
                return;
            }

            // Log success message
            RCLCPP_INFO(this->get_logger(), "Still shot captured and saved at: %s", save_path.c_str());
            response->success = true;
            response->message = "Still shot captured and saved";
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            response->success = false;
            response->message = "cv_bridge exception";
        }
    }
    sensor_msgs::msg::Image::SharedPtr latest_image_msg_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
