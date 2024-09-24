#include "../include/classify.hpp"

namespace classify 
{
void ClassifyModeChoose(cv::Mat& img, const int type)
{
    cv::Mat gray, blurred, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
    // cv::Canny(blurred, edges, 50, 150);
    cv::Canny(gray, edges, 50, 150);
    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), 10);
    std::vector<std::vector<cv::Point>> contours_bef, contours_aft;
    cv::findContours(edges, contours_bef, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Canny(edges, edges, 50, 150);
    cv::findContours(edges, contours_aft, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    double MAXarea = -1;
    std::vector<cv::Point> MAXcontour;
    for(const auto & contour : contours_aft)
    {
        if(contour.empty()) continue;
        if(MAXarea < cv::contourArea(contour)){
            MAXarea = cv::contourArea(contour);
            MAXcontour = contour;
        }
    }
    
    if(MAXcontour.empty()) return;

    double perimeter = cv::arcLength(MAXcontour, true);
    double area = cv::contourArea(MAXcontour);
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
    if(circularity > 0.8)
    {
        cv::Moments m = cv::moments(MAXcontour);
        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
        int radius = static_cast<int>(std::sqrt(area / CV_PI));
        cv::circle(img, center, radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(img, center, 1, cv::Scalar(0, 0, 255), -1);
    }
    // else if(MAXcontour.size() == 4)
    else
    {
        cv::Moments m = cv::moments(MAXcontour);
        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
        int size = static_cast<int>(std::sqrt(area));
        cv::Point top_left(center.x - size / 2, center.y - size / 2);
        cv::Point bottom_right(center.x + size / 2, center.y + size / 2);
        cv::rectangle(img, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
        cv::circle(img, center, 1, cv::Scalar(0, 255, 0), -1);
    }

    // cv::namedWindow("DEBUG", cv::WINDOW_NORMAL);
    // cv::namedWindow("CLASSIFY", cv::WINDOW_NORMAL);
    cv::imshow("DEBUG", edges);
    cv::imshow("CLASSIFY", img);
}












}