#include <iostream>
#include <opencv2/opencv.hpp>
#include "../include/classify.hpp"

int main()
{
    int Counter = 0;
    cv::Mat frame, img;
    cv::VideoCapture capture(0);
    cv::namedWindow("FRAME", cv::WINDOW_NORMAL);
    cv::namedWindow("DEBUG", cv::WINDOW_NORMAL);
    cv::namedWindow("CLASSIFY", cv::WINDOW_NORMAL);
    if (!capture.isOpened()){
        std::cout << "Error opening video stream" << std::endl;
        return -1;
    }

    while(capture.read(frame))
    {
        int type = -1;
        img = frame.clone();
        if(cv::waitKey(30) == '0')
        {
            Counter++;
            // std::cout << Counter << std::endl;
            if(Counter % 2 == 0 || Counter >= 2 * basic::BilliardsNum) {
                type = basic::RubikCubeNum;
                std::cout << Counter << "\t***** Tracking RubikCube *****" << std::endl;
            } else if(Counter % 2 == 1) {
                type = basic::BilliardsNum;
                std::cout << Counter << "\t***** Tracking Billiards *****" << std::endl;
            } if(Counter > 20) return 1;
        } 
        
        classify::ClassifyModeChoose(img, type);
        
        cv::imshow("FRAME", frame);
        cv::waitKey(30);
    }

    cv::destroyAllWindows();
    capture.release();
    return 0;
}