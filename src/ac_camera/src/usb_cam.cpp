#include "usb_cam.hpp"

namespace ac_camera
{
UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & options) : rclcpp::Node("usb_cam", options) 
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());
    
    this->camera_.SensorOpen();
    this->camera_.SensorInit();
    this->image_pub_ = image_transport::create_camera_publisher(this, "/image/raw", rmw_qos_profile_default);
    this->cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/info", 10);
    this->cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "UsbCam");
    auto pkg_path = ament_index_cpp::get_package_share_directory("ac_bringup");
    auto yaml_path = "file://" + pkg_path + "/config/usb_cam_info.yaml";
    if (!cam_info_manager_->loadCameraInfo(yaml_path))
        RCLCPP_WARN(this->get_logger(), "[ %s ] Load camera info fail!", this->get_name());
    else
        cam_info_ = cam_info_manager_->getCameraInfo();
        
    this->capture_thread_ = std::thread{[this]() -> void{
        while(rclcpp::ok())
        {
            if(!this->camera_.isOpen())
            {
                RCLCPP_WARN(this->get_logger(), "[ %s ] Camera is not open!", this->get_name());
                this->camera_.SensorShut();
            }
            cv::Mat image;
            if(this->camera_.SensorRun(image))
            {
                if(!image.empty())
                { 
                    this->image_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                    this->image_->header.stamp = this->now();
                    this->image_->header.frame_id = "camera";

                    this->cam_info_.header.stamp = image_->header.stamp;
                    this->cam_info_.header.frame_id = image_->header.frame_id;

                    this->cam_info_pub_->publish(cam_info_);
                    this->image_pub_->publish(image_, cam_info_);
                }
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