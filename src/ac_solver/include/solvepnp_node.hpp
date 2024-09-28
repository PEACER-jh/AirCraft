#ifndef SOLVE_PNP_HPP
#define SOLVE_PNP_HPP

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

namespace ac_solver
{
class SolvePnPNode : public rclcpp::Node
{
public:
    SolvePnPNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SolvePnPNode();

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    void CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs_;
    sensor_msgs::msg::CameraInfo cam_info_;



private:


};

}

#endif // SOLVE_PNP_HPP