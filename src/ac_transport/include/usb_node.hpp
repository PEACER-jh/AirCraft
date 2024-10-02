#ifndef USB_NODE_HPP
#define USB_NODE_HPP

#include "usb.hpp"

#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <cstdlib>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace ac_transport
{
class UsbNode : public rclcpp::Node
{
public:
    UsbNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~UsbNode();

    rclcpp::CallbackGroup::SharedPtr receive_callback_group_;
    rclcpp::CallbackGroup::SharedPtr target_sub_callback_group_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

public:
    double last_time_;
    rclcpp::Time last_receive_time_;
    rclcpp::TimerBase::SharedPtr receive_timer_;

    void ModeCallBack();
    void PoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
    ReceivePackage receive_package_;
    SendPackage send_package_;
    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    std_msgs::msg::Int8 mode_msg_;
    geometry_msgs::msg::PoseStamped pose_msg_;

public:
    int interface_usb_vid_;
    int interface_usb_pid_;
    int interface_usb_read_endpoint_;
    int interface_usb_write_endpoint_;
    int interface_usb_read_timeout_;
    int interface_usb_write_timeout_;

};

}

#endif // USB_NODE_HPPi