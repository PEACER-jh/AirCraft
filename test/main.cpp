#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat frame;
    cv::VideoCapture capture(0);
    cv::namedWindow("FRAME", cv::WINDOW_NORMAL);
    if (!capture.isOpened()){
        std::cout << "Error opening video stream" << std::endl;
        return -1;
    }

    while(capture.read(frame))
    {
        cv::imshow("FRAME", frame);
        cv::waitKey(30);
    }

    return 0;
}