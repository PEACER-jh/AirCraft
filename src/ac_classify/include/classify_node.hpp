#ifndef CLASSIFY_NODE_HPP
#define CLASSIFY_NODE_HPP

#include <vector>
#include <iostream>
#include <unordered_map>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/publisher.hpp"
#include "image_transport/subscriber.hpp"
#include "image_transport/image_transport.hpp"
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

    void ImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img);
    void CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
    void ImagePub(std_msgs::msg::Header header);

private:
    cv::Mat image_;
    cv::Mat image_debug_, image_mark_;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    sensor_msgs::msg::CameraInfo cam_info_;

    std::unordered_map<ObjectType, int> ObjectType_;
    std::unordered_map<ObjectColor, int> ObjectColor_;
    std::vector<Object> objects_;

};

}

#endif // CLASSIFY_NODE_HPP