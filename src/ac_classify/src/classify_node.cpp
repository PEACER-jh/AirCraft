#include "classify_node.hpp"

namespace ac_classify 
{
ClassifyNode::ClassifyNode(const rclcpp::NodeOptions & options) : rclcpp::Node("classify_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    cv::namedWindow("Gauss", cv::WINDOW_FREERATIO);
    cv::namedWindow("Canny", cv::WINDOW_FREERATIO);

    this->min_area_ = this->declare_parameter<int>("min_area", 5000);
    this->max_area_ = this->declare_parameter<int>("max_area", 50000);
    this->offset_u_ = this->declare_parameter<int>("offset_u", 0);
    this->offset_v_ = this->declare_parameter<int>("offset_v", 0);
    this->max_contour_number = this->declare_parameter<int>("max_contour_number", 5);

    this->bin_low_threshold_ = this->declare_parameter<int>("bin_low_threshold", 50);
    this->bin_high_threshold_ = this->declare_parameter<int>("bin_high_threshold", 255);
    this->first_canny_low_threshold_ = this->declare_parameter<int>("first_canny_low_threthold", 50);
    this->first_canny_high_threshold_ = this->declare_parameter<int>("first_canny_high_threthold", 150);
    this->second_canny_low_threshold_ = this->declare_parameter<int>("second_canny_low_threthold", 50);
    this->second_canny_high_threshold_ = this->declare_parameter<int>("second_canny_high_threthold", 150);
    this->dilate_iterations_ = this->declare_parameter<int>("dilate_iterations", 5);
    this->hough_min_radius_ = this->declare_parameter<int>("hough_min_radius", 5);
    this->hough_max_radius_ = this->declare_parameter<int>("hough_max_radius", 100);
    this->hough_canny_threshold_ = this->declare_parameter<int>("hough_canny_threshold", 20);

    this->image_mark_pub_ = image_transport::create_camera_publisher(this, "/image_mark", rmw_qos_profile_default);
    this->image_debug_pub_ = image_transport::create_camera_publisher(this, "/image_debug", rmw_qos_profile_default);
    this->contour_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/contour", 10);

    this->object_type_sub_ = this->create_subscription<std_msgs::msg::Int8>
                ("/mode", rclcpp::SensorDataQoS(), std::bind(&ClassifyNode::ObjectCallBack, this, std::placeholders::_1));
    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
                ("/camera/info", rclcpp::SensorDataQoS(), std::bind(&ClassifyNode::CameraInfoCallBack, this, std::placeholders::_1));
    this->image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ClassifyNode::ImageCallBack, this, std::placeholders::_1),
                "raw", rmw_qos_profile_default));

}


ClassifyNode::~ClassifyNode()
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

void ClassifyNode::ImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
    cv::Mat gray, blurred, bin, edges, mark;
    std::vector<std::vector<cv::Point>> contours;
    this->image_ = cv_bridge::toCvShare(img, "bgr8")->image;
    mark = this->image_.clone();
    this->arm_center_ = cv::Point(mark.cols / 2 + offset_u_, mark.rows / 2 + offset_v_);

    /* 转变成为灰度图 */    cv::cvtColor(this->image_, gray, cv::COLOR_BGR2GRAY);
    if(this->object_type_ == (int)ObjectType::RUBIKCUBE)
    {
    /* 高斯滤波器模糊 */    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1);
    /* 为魔方边缘检测 */    cv::Canny(blurred, edges, first_canny_low_threshold_, first_canny_high_threshold_);
    /* 对图像膨胀操作 */    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), dilate_iterations_);
    /* 在图像寻找轮廓 */    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
    if(this->object_type_ == (int)ObjectType::BILLIARDS)
    {
    // alpha 控制对比度（>1 增强对比度），beta 控制亮度
        gray.convertTo(gray, -1, 2.0, -50);  // 增加对比度，减少亮度
    /* 高斯滤波器模糊 */    cv::GaussianBlur(gray, blurred, cv::Size(1, 1), 0);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(blurred, bin, cv::MORPH_CLOSE, kernel);
    /* 为台球边缘检测 */    cv::Canny(bin, edges, first_canny_low_threshold_, first_canny_high_threshold_);
    // /* 对图像膨胀操作 */    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), dilate_iterations_);
    /* 在图像寻找轮廓 */    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
    
    cv::imshow("Gauss", blurred);
    cv::imshow("Canny", edges);
    cv::waitKey(30);

    // 目标选取策略——在最大的前几个轮廓中选取质心距手眼标定处最近的轮廓
    std::sort(contours.begin(), contours.end(), 
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b){return cv::contourArea(a) > cv::contourArea(b);});
    switch(this->object_type_)
    {
        case (int)ObjectType::RUBIKCUBE: { findRubikCube(mark, contours); break; }
        case (int)ObjectType::BILLIARDS: { findBilliards2(mark, edges); break; }
        // case (int)ObjectType::BILLIARDS: { findBilliards(mark, contours); break; }
        default: break;
    }
    
    cv::circle(mark, arm_center_, 3, cv::Scalar(0, 255, 0), -1);
    auto header = img->header;
    this->image_mark_ = mark;
    this->image_debug_ = edges;
    this->ImagePub(header);
}

void ClassifyNode::ObjectCallBack(const std_msgs::msg::Int8::SharedPtr msg)
{
    this->object_type_ = (int)msg->data;
    if((int)msg->data != this->object_type_)
    {   
        auto getType = [](int value){
            switch(value)
            {
                case (int)ObjectType::RUBIKCUBE: return "RubikCube";
                case (int)ObjectType::BILLIARDS: return "Billiards";
                default:                         return "Unknown";
            }};
        this->polygons_.header.frame_id = getType(msg->data);
        RCLCPP_INFO(this->get_logger(), "****** Receive object type : %s ******", this->polygons_.header.frame_id);
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
    // debug 图像发布
    sensor_msgs::msg::Image::SharedPtr debug;
    debug = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", this->image_debug_).toImageMsg();
    debug->header = header;
    this->image_debug_pub_.publish(*debug, this->cam_info_);

    // mark 图像发布
    sensor_msgs::msg::Image::SharedPtr mark;
    mark = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", this->image_mark_).toImageMsg();
    mark->header = header;
    this->image_mark_pub_.publish(*mark, this->cam_info_);
}

void ClassifyNode::ContourPub(cv::Mat &mark, std::vector<cv::Point> contour)
{
    this->polygons_.polygon.points.clear();
    this->contour_ = contour;

    // 为发布消息赋frame_id，为解算节点作判断作准备
    this->polygons_.header.frame_id = "None";
    this->polygons_.header.stamp = this->now();
    if(this->object_type_ == (int)ObjectType::RUBIKCUBE)
        this->polygons_.header.frame_id = "RubikCube";   
    else if(this->object_type_ == (int)ObjectType::BILLIARDS)
        this->polygons_.header.frame_id = "Billiards";

    geometry_msgs::msg::Point32 p;
    if(this->object_type_ == (int)ObjectType::RUBIKCUBE)    // 魔方边框处理
    {
        // 四边形的四个顶点
       for(auto point : contour_)
       {
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            polygons_.polygon.points.push_back(p);
       }
    }
    if(this->object_type_ == (int)ObjectType::BILLIARDS)    // 台球边框处理
    {
        // 圆形最小外接矩形的四个顶点
        cv::Rect rect = cv::boundingRect(contour);
        int length = std::min(rect.width, rect.height);

        p.x = rect.x;   p.y = rect.y;   p.z = 0;
        polygons_.polygon.points.push_back(p);
        p.x = rect.x + length;   p.y = rect.y;   p.z = 0;
        polygons_.polygon.points.push_back(p);
        p.x = rect.x;   p.y = rect.y + length;   p.z = 0;
        polygons_.polygon.points.push_back(p);
        p.x = rect.x + length;   p.y = rect.y + length;   p.z = 0;
        polygons_.polygon.points.push_back(p);

        cv::line(mark, cv::Point(rect.x, rect.y), cv::Point(rect.x + length, rect.y), cv::Scalar(0, 0, 255), 2);
        cv::line(mark, cv::Point(rect.x + length, rect.y), cv::Point(rect.x + length, rect.y + length), cv::Scalar(0, 0, 255), 2);
        cv::line(mark, cv::Point(rect.x + length, rect.y + length), cv::Point(rect.x, rect.y + length), cv::Scalar(0, 0, 255), 2);
        cv::line(mark, cv::Point(rect.x, rect.y + length), cv::Point(rect.x, rect.y), cv::Scalar(0, 0, 255), 2);
    }
    this->contour_pub_->publish(this->polygons_);
}

// 寻找魔方轮廓
void ClassifyNode::findRubikCube(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours)
{
    int count = 0;
    double MAXarea = -1;
    std::vector<cv::Point> MAXcontour;
    std::vector<std::vector<cv::Point>> choose_contours = {};
    this->objects_.clear();
    
    for(auto & contour : contours)
    {
        if(contour.empty()) continue;
        if(count > this->max_contour_number || count > RubikCubeNum) break;
    
        std::vector<cv::Point> approx = {};
        cv::approxPolyDP(contour, approx, cv::arcLength(cv::Mat(contour), true) * 0.02, true);
        // if(approx.size() == 4)
        if(approx.size() <= 6)
        {
            double area = cv::contourArea(approx);
            if(area < min_area_ || area > max_area_) continue;
            if(MAXarea < cv::contourArea(approx))
            {
                MAXarea = cv::contourArea(approx);
                MAXcontour = approx;
            }
            
            Object object(ObjectType::RUBIKCUBE, ObjectColor::NONE, approx);
            choose_contours.push_back(approx);
            this->objects_.push_back(object);
            count++;
        }
    }

    auto choose = chooseObject(mark, choose_contours);
    if(!choose.empty())
    {   
        this->ContourPub(mark ,choose);
        
        cv::Moments m = cv::moments(choose);
        cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
        std::vector<cv::Point> app;
        cv::approxPolyDP(choose, app, cv::arcLength(cv::Mat(choose), true) * 0.02, true);
        cv::drawContours(mark, std::vector<std::vector<cv::Point>>{app}, 0, cv::Scalar(0, 0, 255), 2);
        cv::line(mark, center, arm_center_, cv::Scalar(0, 255, 0), 2);
    } else {
        RCLCPP_WARN(this->get_logger(), "***************** No object found *****************");
    }
}

// 寻找台球轮廓
void ClassifyNode::findBilliards(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours)
{
    int count = 0;
    double MAXarea = -1;
    std::vector<cv::Point> MAXcontour;
    std::vector<std::vector<cv::Point>> choose_contours = {};
    this->objects_.clear();
    
    double perimeter = 0, area = 0, circularity = 0;
    for(auto & contour : contours)
    {
        if(contour.empty()) continue;
        if(count > this->max_contour_number || count > RubikCubeNum) continue;;
    
        perimeter = cv::arcLength(contour, true);
        area = cv::contourArea(contour);
        circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if(circularity > 0.8)
        {
            if(area < min_area_ || area > max_area_) continue;
            if(MAXarea < area)
            {
                MAXarea = area;
                MAXcontour = contour;
            }
            // TODO : 台球颜色识别——在HSV色彩空间中寻找最近的”距离“
            // auto color = this->recogizeColor(mark, contour);
            Object object(ObjectType::BILLIARDS, ObjectColor::NONE, contour);
            choose_contours.push_back(contour);
            this->objects_.push_back(object);
            count++;
        } 
    }

    auto choose = chooseObject(mark, choose_contours);
    if(!choose.empty())
    {
        this->ContourPub(mark, choose);
        
        cv::Moments m = cv::moments(choose);
        cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
        double circle_area = cv::contourArea(choose);
        int size = static_cast<int>(std::sqrt(circle_area / CV_PI));
        cv::circle(mark, center, size, cv::Scalar(0, 0, 255), 2);
        cv::line(mark, center, arm_center_, cv::Scalar(0, 255, 0), 2);
    } else {
        RCLCPP_WARN(this->get_logger(), "***************** No object found *****************");
    }

}

void ClassifyNode::findBilliards2(cv::Mat& mark, cv::Mat& edge)
{
        // 霍夫圆检测
        int min_radius = this->hough_min_radius_;
        int max_radius = this->hough_max_radius_;
        int canny_threshold = this->hough_canny_threshold_;
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(edge, circles, cv::HOUGH_GRADIENT, 1, edge.rows/8, 100, canny_threshold, min_radius, max_radius);
        
        std::vector<std::vector<cv::Point>> contours;
        // 遍历每个检测到的圆
        for (size_t i = 0; i < circles.size(); i++) 
        {
            cv::Vec3f circle = circles[i];
            float x_center = circle[0];  // 圆心 x 坐标
            float y_center = circle[1];  // 圆心 y 坐标
            float radius = circle[2];    // 圆的半径
            // 用于存储该圆的轮廓点
            std::vector<cv::Point> contour; 
            // 根据极坐标生成圆周上的点
            for (int angle = 0; angle < 360; angle++) 
            {
                float theta = angle * CV_PI / 180.0;  // 将角度转换为弧度
                int x = std::round(x_center + radius * std::cos(theta));  // 计算圆周上的 x 坐标
                int y = std::round(y_center + radius * std::sin(theta));  // 计算圆周上的 y 坐标
                contour.push_back(cv::Point(x, y));  // 将点添加到轮廓中
            }
            // 将生成的轮廓添加到 contours 列表中
            contours.push_back(contour);
        }
        auto choose = chooseObject(mark, contours); 
        if(!choose.empty())
        {
            this->ContourPub(mark, choose);

            cv::Moments m = cv::moments(choose);
            cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
            double circle_area = cv::contourArea(choose);
            int size = static_cast<int>(std::sqrt(circle_area / CV_PI));
            cv::circle(mark, center, size, cv::Scalar(0, 0, 255), 2);
            cv::line(mark, center, arm_center_, cv::Scalar(0, 255, 0), 2);
        } else {
            RCLCPP_WARN(this->get_logger(), "***************** No object found *****************");
        }
    
}

// 筛选出最后的目标轮廓
std::vector<cv::Point> ClassifyNode::chooseObject(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours)
{
    double min_dist = INFINITY;
    std::vector<cv::Point> choose = {};
    int center_x = mark.cols / 2 + this->offset_u_;
    int center_y = mark.rows / 2 + this->offset_v_;
    for(auto & contour : contours)
    {
        if(contour.empty()) continue;
        cv::Moments m = cv::moments(contour);
        cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
        double dist = std::sqrt((center.x - center_x) * (center.x - center_x) + (center.y - center_y) * (center.y - center_y));
        if(dist < min_dist)
        {
            min_dist = dist;
            choose = contour;
        }
    }
    return choose;
}

ObjectColor ClassifyNode::recogizeColor(cv::Mat& mark, std::vector<cv::Point>& contour)
{
    cv::Mat mask = cv::Mat::zeros(mark.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<cv::Point>{contour}, -1, cv::Scalar(255), cv::FILLED);
    cv::Mat roi = mark & mask;
    // TODO : 颜色识别
    return ObjectColor::NONE;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_classify::ClassifyNode)