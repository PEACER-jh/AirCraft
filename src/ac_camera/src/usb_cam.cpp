#include "usb_cam.hpp"

namespace ac_camera
{
UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & options) : rclcpp::Node("usb_cam", options) 
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());
    
    this->camera_.SensorOpen();
    this->camera_.SensorInit();
    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image/raw", 10);
    this->capture_thread_ = std::thread{[this]() -> void{
        while(rclcpp::ok())
        {
            cv::Mat image;
            cv_bridge::CvImage bridge;
            this->camera_.SensorRun(image);
            if(!image.empty()){
                bridge.header.stamp = this->now();
                bridge.header.frame_id = "camera";
                bridge.encoding = "bgr8";
                bridge.image = image;   
                this->image_pub_->publish(*bridge.toImageMsg());
            }
        }
    }};

}   

UsbCamNode::~UsbCamNode() 
{
    if(capture_thread_.joinable())
        capture_thread_.join();
    this->camera_.SensorShut();
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_camera::UsbCamNode)