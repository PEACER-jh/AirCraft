#ifndef AC_USB_CAM_HPP
#define AC_USB_CAM_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"

#include "cam_interface.hpp"

namespace ac_camera
{
class UsbCamNode : public rclcpp::Node
{
public:
    UsbCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~UsbCamNode();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager;

    CamInterface camera_;
    sensor_msgs::msg::Image image_;
    sensor_msgs::msg::CameraInfo cam_info_;
    std::thread capture_thread_;

};

}

#endif //AC_USB_CAM_HPP