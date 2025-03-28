#ifndef _TRANSFORMFRAME_H_
#define _TRANSFORMFRAME_H_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "LogTime.hpp"

bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2,
    std::ofstream& file_output // added 2025
    );
bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2);

#endif