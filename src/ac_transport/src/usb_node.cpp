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
    
    this->callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UsbNode::parametersCallback, this, std::placeholders::_1));

    // 初始化usb
    this->interface_usb_vid_ = this->declare_parameter<int>("interface_usb_vid", 0x0483);
    this->interface_usb_pid_ = this->declare_parameter<int>("interface_usb_pid", 0x5740);
    this->interface_usb_read_endpoint_ = this->declare_parameter<int>("interface_usb_read_endpoint", 0x81);
    this->interface_usb_write_endpoint_ = this->declare_parameter<int>("interface_usb_write_endpoint", 0x01);
    this->interface_usb_read_timeout_ = this->declare_parameter<int>("interface_usb_read_timeout", 1);
    this->interface_usb_write_timeout_ = this->declare_parameter<int>("interface_usb_write_timeout", 1);
    this->transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
        this->interface_usb_vid_, 
        this->interface_usb_pid_, 
        this->interface_usb_read_endpoint_, 
        this->interface_usb_write_endpoint_, 
        this->interface_usb_read_timeout_, 
        this->interface_usb_write_timeout_
    );
    if (transporter_->open() == true) 
        RCLCPP_INFO(this->get_logger(), "Open usb transport SUCCESS!!!");
    else
        RCLCPP_INFO(this->get_logger(), "Open usb transport FAILED!!!");
    RCLCPP_INFO(this->get_logger(), "Finish Init");
}

UsbNode::~UsbNode()
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

void UsbNode::PoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{ 
    this->send_package_._SOF = 0x55;
    this->send_package_.ID = SEND_ID;
    this->send_package_._EOF = 0xFF;
    std::memcpy(&this->send_package_.x[0], &pose->pose.position.x, sizeof(float));
    std::memcpy(&this->send_package_.y[0], &pose->pose.position.y, sizeof(float));
    
    this->transporter_->write((unsigned char *)&this->send_package_, sizeof(SendPackage));
}

void UsbNode::ModeCallBack()
{
    // const int size = sizeof(ReceivePackage);
    const int size = 64;
    uint8_t receive[size];
    int read_size = this->transporter_->read(receive, size);

    if(receive[1] == RECEIVE_ID)
    {
        ReceivePackage package;
        std::memcpy(&package, receive, sizeof(ReceivePackage));
        auto mode = package.mode & 0x01;
        RCLCPP_INFO(this->get_logger(), "[ %s ] receive mode from aircraft: %d", this->get_name(), mode);

        this->mode_pub_->publish(mode);
    }
}

rcl_interfaces::msg::SetParametersResult UsbNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for(const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());

        if(param.get_name() == "/mode")
            this->object_type_ = (ObjectType)param.as_int();
    }
    return result;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ac_transport::UsbNode)