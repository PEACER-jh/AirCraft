#ifndef SOLVE_PNP_HPP
#define SOLVE_PNP_HPP

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

namespace ac_solver
{
#define RubikCubeSize 0.560   // m
#define BilliardsSize 0.525   // m

class SolvePnPNode : public rclcpp::Node
{
public:
    SolvePnPNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SolvePnPNode();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr contour_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    void initSolver();
    void CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
    void ContourCallBack(const geometry_msgs::msg::PolygonStamped::SharedPtr contour);

private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs_;
    sensor_msgs::msg::CameraInfo cam_info_;

private:
    double arm_offset_x_;
    double arm_offset_y_;
    std::vector<cv::Point2d> contour_;
    std::vector<cv::Point3d> rubikcube_;
    std::vector<cv::Point3d> billiards_;
    geometry_msgs::msg::PoseStamped pose_;

};

}

#endif // SOLVE_PNP_HPP