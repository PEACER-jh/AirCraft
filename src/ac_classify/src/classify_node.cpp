#include "classify_node.hpp"

namespace ac_classify
{
ClassifyNode::ClassifyNode(const rclcpp::NodeOptions & options) : rclcpp::Node("classify_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    this->image_mark_pub_ = image_transport::create_camera_publisher(this, "/image_mark", rmw_qos_profile_default);
    this->image_debug_pub_ = image_transport::create_camera_publisher(this, "/image_debug", rmw_qos_profile_default);

    image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ClassifyNode::ImageCallBack, this, std::placeholders::_1),
                "raw", rmw_qos_profile_default));
    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
                ("/camera/info", rclcpp::SensorDataQoS(), std::bind(&ClassifyNode::CameraInfoCallBack, this, std::placeholders::_1));
}


ClassifyNode::~ClassifyNode()
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

void ClassifyNode::ImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
    cv::Mat gray, blurred, edges, mark;
    std::vector<std::vector<cv::Point>> contours1, contours2;
    this->image_ = cv_bridge::toCvShare(img, "bgr8")->image;

    cv::cvtColor(this->image_, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 50, 150);
    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), 10);
    cv::findContours(edges, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Canny(edges, edges, 50, 150);
    cv::findContours(edges, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);





    auto header = img->header;
    // this->image_mark_ = mark.clone();
    this->image_debug_ = edges.clone();
    this->ImagePub(header);
}

void ClassifyNode::CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    RCLCPP_INFO(this->get_logger(), "[ %s ] Receive camera infomation", this->get_name());
    this->cam_info_ = *info;
    this->camera_matrix.create(3, 3, CV_64FC1);
    this->dist_coeffs_.create(1, 5, CV_64FC1);
    for(int i = 0; i < 9; i++)
        this->camera_matrix.at<double>(i / 3, 1 % 3) = this->cam_info_.k[i];
    for(int i = 0; i < 5; i++)
        this->dist_coeffs_.at<double>(0, i) = this->cam_info_.d[i];
    this->camera_info_sub_.reset();
}

void ClassifyNode::ImagePub(std_msgs::msg::Header header)
{
    sensor_msgs::msg::Image::SharedPtr debug;
    debug = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", this->image_debug_).toImageMsg();
    debug->header = header;
    this->image_debug_pub_.publish(*debug, this->cam_info_);

}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_classify::ClassifyNode)