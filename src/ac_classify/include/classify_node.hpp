#ifndef CLASSIFY_NODE_HPP
#define CLASSIFY_NODE_HPP

#include <iostream>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace ac_classify
{
enum class ObjectType
{
    RUBIKCUBE,
    BILLIARDS,
};

enum class ObjectColor
{
    YELLOW,
    GREEN,
    BROWN,
    BULE,
    PINK,
    BLACK,
    NONE,
};

class ClassifyNode : public rclcpp::Node
{
public:
    ClassifyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ClassifyNode();


private:


};

}

#endif // CLASSIFY_NODE_HPP