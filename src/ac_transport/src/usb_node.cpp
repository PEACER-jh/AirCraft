#include "usb_node.hpp"

namespace ac_transport
{
UsbNode::UsbNode(const rclcpp::NodeOptions &options) : rclcpp::Node("usb_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    this->receive_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->target_sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto target_sub_options = rclcpp::SubscriptionOptions();
    target_sub_options.callback_group = this->target_sub_callback_group_;
    this->pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
                ("/pose", rclcpp::SensorDataQoS(), std::bind(&UsbNode::PoseCallBack, this, std::placeholders::_1), target_sub_options);

    this->receive_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UsbNode::ModeCallBack, this));
    this->mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("/mode", 10);

    interface_usb_vid_ = this->declare_parameter("interface_usb_vid", 0x0483);
    interface_usb_pid_ = this->declare_parameter("interface_usb_pid", 0x5740);
    interface_usb_read_endpoint_ = this->declare_parameter("interface_usb_read_endpoint", 0x81);
    interface_usb_write_endpoint_ = this->declare_parameter("interface_usb_write_endpoint", 0x01);
    interface_usb_read_timeout_ = this->declare_parameter("interface_usb_read_timeout", 1);
    interface_usb_write_timeout_ = this->declare_parameter("interface_usb_write_timeout", 1);
    this->transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
        this->interface_usb_vid_, 
        this->interface_usb_pid_, 
        this->interface_usb_read_endpoint_, 
        this->interface_usb_write_endpoint_, 
        this->interface_usb_read_timeout_, 
        this->interface_usb_write_timeout_
    );

    this->callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UsbNode::parametersCallback, this, std::placeholders::_1));

}

UsbNode::~UsbNode()
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ac_transport::UsbNode)