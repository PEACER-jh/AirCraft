#ifndef AC_CLASSIFY_NODE_HPP
#define AC_CLASSIFY_NODE_HPP

#include <vector>
#include <iostream>
#include <unordered_map>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/publisher.hpp"
#include "image_transport/subscriber.hpp"
#include "image_transport/image_transport.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "classify_basic.hpp"

namespace ac_classify
{
class ClassifyNode : public rclcpp::Node
{
public:
    ClassifyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ClassifyNode();

    image_transport::CameraPublisher image_mark_pub_;
    image_transport::CameraPublisher image_debug_pub_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    void ImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img);            // 读取图像
    void CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);   // 读取相机内参
    void ImagePub(std_msgs::msg::Header header);        // 发布图像
    void ContourPub(std::vector<cv::Point> contour);    // 发布轮廓

private:
    cv::Mat image_;
    cv::Mat image_mark_;
    cv::Mat image_debug_;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    sensor_msgs::msg::CameraInfo cam_info_;

public:
    std::unordered_map<ObjectType, int> ObjectType_;
    std::unordered_map<ObjectColor, int> ObjectColor_;
    std::vector<Object> objects_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr object_type_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr contour_pub_;
    void ObjectCallBack(const std_msgs::msg::Int8::SharedPtr msg);
    void findRubikCube(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours);                   // 寻找魔方
    void findBilliards(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours);                   // 寻找台球
    std::vector<cv::Point> chooseObject(cv::Mat& mark, std::vector<std::vector<cv::Point>>& contours);  // 选择对象
    ObjectColor recogizeColor(cv::Mat& mark, std::vector<cv::Point>& contour);                          // 识别颜色(TODO)

private:
    int object_type_ = (int)ObjectType::RUBIKCUBE;
    std::vector<cv::Point> contour_;
    geometry_msgs::msg::PolygonStamped polygons_;

    double min_area_;   // 识别的最小面积
    double max_area_;   // 识别的最大面积

    int offset_u_;  // 图像中心点x方向（行）偏移量
    int offset_v_;  // 图像中心点y方向（列）偏移量
    cv::Point arm_center_;  // 手眼标定中心点

    int first_canny_low_threshold_;     // 第一级canny边缘检测低阈值
    int first_canny_high_threshold_;    // 第一级canny边缘检测高阈值
    int second_canny_low_threshold_;    // 第二级canny边缘检测低阈值
    int second_canny_high_threshold_;   // 第二级canny边缘检测高阈值
    int dilate_iterations_;             // 膨胀迭代次数
    int max_contour_number;             // 最大轮廓数量

};

}

#endif // AC_CLASSIFY_NODE_HPP