#ifndef CLASSIFY_HPP
#define CLASSIFY_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../include/basic.hpp"

namespace classify
{
void ClassifyModeChoose(cv::Mat& img, const int type);

// void RubikcubeClassify(cv::Mat& img, std::vector<cv::Point>& points);
// void BilliardsClassify(cv::Mat& img, std::vector<cv::Point>& points);

// void ColorClassify(cv::Mat& img, std::vector<cv::Point>& points);
} 

#endif // CLASSIFY_HPP