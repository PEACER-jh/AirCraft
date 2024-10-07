#ifndef AC_USB_CAM_HPP
#define AC_USB_CAM_HPP

#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "cam_interface.hpp"

namespace ac_camera
{
class UsbCamNode : public rclcpp::Node
{
public:
    UsbCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~UsbCamNode();

private:
    image_transport::CameraPublisher image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

    int count_ = 0;
    CamInterface camera_;
    sensor_msgs::msg::Image::SharedPtr image_;
    sensor_msgs::msg::CameraInfo cam_info_;
    std::thread capture_thread_;

};

}

#endif //AC_USB_CAM_HPP