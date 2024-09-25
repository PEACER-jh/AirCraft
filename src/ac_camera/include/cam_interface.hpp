#ifndef AC_CAM_INTERFACE_HPP_
#define AC_CAM_INTERFACE_HPP_

#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace ac_camera
{
enum class CamParamsEnum
{
  Width,                  // 图像宽度
  Height,                 // 图像高度 
  AutoExposure,           // 自动曝光
  Exposure,               // 曝光时间 
  Brightness,             // 亮度
  AutoWhiteBalance,       // 自动白平衡
  WhiteBalance,           // 白平衡
  Gain,                   // 增益
  RGain,                  // 红增益
  GGain,                  // 绿增益
  BGain,                  // 蓝增益
  Gamma,                  // 伽马
  Contrast,               // 对比度
  Saturation,             // 饱和度
  Hue,                    // 色调
  Fps                     // 帧率
};

class CamInterface
{
public:
  CamInterface();
  ~CamInterface();

  bool SensorOpen();
  bool SensorInit();
  bool SensorRun(cv::Mat & image);
  bool SensorShut();
  
  bool isOpen();
  bool isInit();
  bool isRun();

  // bool set_parameter(CamParamsEnum type, int value);
  // bool get_parameter(CamParamsEnum type, int & value);
  // std::string info_error();

private:
  bool is_open_;
  bool is_init_;
  bool is_run_;
  cv::VideoCapture capture_;
  std::string error_message_;
  std::unordered_map<ac_camera::CamParamsEnum, int> cam_params_;

  // Eigen::Mac

};

}

#endif // AC_CAM_INTERFACE_HPP_