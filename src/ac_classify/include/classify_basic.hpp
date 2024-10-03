#ifndef AC_CLASSIFY_BASIC_HPP
#define AC_CLASSIFY_BASIC_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace ac_classify
{
#define RubikCubeSize 0.560   // m
#define BilliardsSize 0.525   // m
#define RubikCubeNum 15
#define BilliardsNum 6

enum class ObjectType
{
    RUBIKCUBE,  // 魔方 - 0
    BILLIARDS,  // 台球 - 1
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

struct Object
{
    double size_;
    ObjectType type_;
    ObjectColor color_;
    std::vector<cv::Point> contour_;

    Object(ObjectType type, ObjectColor color, std::vector<cv::Point> contour) : type_(type)
    {
        contour_ = contour;
        if(type_ == ObjectType::RUBIKCUBE){
            size_ = RubikCubeSize;
            color_ = ObjectColor::NONE;
        } else if(type_ == ObjectType::BILLIARDS) {
            size_ = BilliardsSize;
            color_ = color;
        } else {
            return;
        }
    }
};

// 定义颜色范围 HSV
cv::Scalar lower_yellow(20, 100, 100);
cv::Scalar upper_yellow(30, 255, 255);  // 黄色
cv::Scalar lower_green(50, 100, 100);
cv::Scalar upper_green(70, 255, 255);   // 绿色
cv::Scalar lower_brown(0, 0, 0);
cv::Scalar upper_brown(10, 100, 100);   // 棕色
cv::Scalar lower_blue(110, 100, 100);
cv::Scalar upper_blue(130, 255, 255);   // 蓝色
cv::Scalar lower_pink(145, 100, 100);
cv::Scalar upper_pink(170, 255, 255);   // 粉色
cv::Scalar lower_black(0, 0, 0);
cv::Scalar upper_black(180, 255, 30);   // 黑色

}

#endif // AC_CLASSIFY_BASIC_HPP