#include "classify_node.hpp"

namespace ac_classify
{
ClassifyNode::ClassifyNode(const rclcpp::NodeOptions & options) : rclcpp::Node("classify_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    this->first_canny_low_threshold_ = this->declare_parameter<int>("first_canny_low_threthold", 50);
    this->first_canny_high_threshold_ = this->declare_parameter<int>("first_canny_high_threthold", 150);
    this->second_canny_low_threshold_ = this->declare_parameter<int>("second_canny_low_threthold", 50);
    this->second_canny_high_threshold_ = this->declare_parameter<int>("second_canny_high_threthold", 150);
    this->dilate_iterations_ = this->declare_parameter<int>("dilate_iterations", 5);
    this->max_contour_number = this->declare_parameter<int>("max_contour_number", 5);

    this->image_mark_pub_ = image_transport::create_camera_publisher(this, "/image_mark", rmw_qos_profile_default);
    this->image_debug_pub_ = image_transport::create_camera_publisher(this, "/image_debug", rmw_qos_profile_default);

    this->object_type_sub_ = this->create_subscription<std_msgs::msg::Int8>
                ("/object", rclcpp::SensorDataQoS(), std::bind(&ClassifyNode::ObjectCallBack, this, std::placeholders::_1));
    this->image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
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
    mark = this->image_.clone();

    /* 转变成为灰度图 */    cv::cvtColor(this->image_, gray, cv::COLOR_BGR2GRAY);
    /* 第一级边缘检测 */    cv::Canny(gray, edges, first_canny_low_threshold_, first_canny_high_threshold_);
    /* 对图像膨胀操作 */    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), dilate_iterations_);
    /* 第一级寻找轮廓 */    cv::findContours(edges, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    /* 第二级边缘检测 */    cv::Canny(edges, edges, second_canny_low_threshold_, second_canny_high_threshold_);
    /* 第二级寻找轮廓 */    cv::findContours(edges, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::sort(contours2.begin(), contours2.end(), 
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b){return cv::contourArea(a) > cv::contourArea(b);});
    /* TODO : test */ this->object_type_ = 1;
    switch(this->object_type_)
    {
        case (int)ObjectType::RUBIKCUBE: {findRubikCube(mark, contours2); break;}
        case (int)ObjectType::BILLIARDS: {findBilliards(mark, contours2); break;}
        default: break;
    }

    auto header = img->header;
    this->image_mark_ = mark;
    this->image_debug_ = edges;
    this->ImagePub(header);

}

void ClassifyNode::ObjectCallBack(const std_msgs::msg::Int8::SharedPtr msg)
{
    if(msg->data != this->object_type_)
    {   
        auto getType = [](int value){
            switch(value)
            {
                // TODO ： 目标选取策略——在最大的前几个轮廓中选取质心距手眼标定处最近的轮廓
                case (int)ObjectType::RUBIKCUBE: return "RubikCube";
                case (int)ObjectType::BILLIARDS: return "Billiards";
                default:                         return "Unknown";
            }};
        RCLCPP_INFO(this->get_logger(), "****** Receive object type : %s ******", getType(msg->data));
    }
    
    this->object_type_ = msg->data;
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

    sensor_msgs::msg::Image::SharedPtr mark;
    mark = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", this->image_mark_).toImageMsg();
    mark->header = header;
    this->image_mark_pub_.publish(*mark, this->cam_info_);
}

void ClassifyNode::findRubikCube(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours)
{
    int count = 0;
    double MAXarea = -1;
    std::vector<cv::Point> MAXcontour;
    this->objects_.clear();
    
    for(auto & contour : contours)
    {
        if(contour.empty()) continue;
        if(count > this->max_contour_number || count > RubikCubeNum) break;
    
        std::vector<cv::Point> approx = {};
        cv::approxPolyDP(contour, approx, cv::arcLength(cv::Mat(contour), true) * 0.02, true);
        if(approx.size() == 4)
        {
            if(MAXarea < cv::contourArea(approx))
            {
                MAXarea = cv::contourArea(approx);
                MAXcontour = approx;
            }
            
            Object object(ObjectType::RUBIKCUBE, ObjectColor::NONE, approx);
            this->objects_.push_back(object);
            count++;
        }
    }
    
    if(!MAXcontour.empty())
        cv::drawContours(mark, std::vector<std::vector<cv::Point>>{MAXcontour}, 0, cv::Scalar(0, 0, 255), 2);
}

void ClassifyNode::findBilliards(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours)
{
    int count = 0;
    double MAXarea = -1;
    std::vector<cv::Point> MAXcontour;
    this->objects_.clear();
    
    double perimeter = 0, area = 0, circularity = 0;
    for(auto & contour : contours)
    {
        if(contour.empty()) continue;
        if(count > this->max_contour_number || count > RubikCubeNum) break;
    
        perimeter = cv::arcLength(contour, true);
        area = cv::contourArea(contour);
        circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if(circularity > 0.8)
        {
            if(MAXarea < area)
            {
                MAXarea = area;
                MAXcontour = contour;
            }
            // TODO : 台球颜色识别 
            auto color = this->recogizeColor(mark, contour);
            Object object(ObjectType::BILLIARDS, color, contour);
            this->objects_.push_back(object);
            count++;
        } 
    }

    cv::Moments m = cv::moments(MAXcontour);
    cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
    int size = static_cast<int>(std::sqrt(MAXarea / CV_PI));
    if(!MAXcontour.empty())
        cv::circle(mark, center, size, cv::Scalar(0, 0, 255), 2);
    
}

ObjectColor ClassifyNode::recogizeColor(cv::Mat& mark, std::vector<cv::Point>& contour)
{
    cv::Mat mask = cv::Mat::zeros(mark.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<cv::Point>{contour}, -1, cv::Scalar(255), cv::FILLED);
    cv::Mat roi = mark & mask;


}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_classify::ClassifyNode)