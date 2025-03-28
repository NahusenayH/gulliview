#ifndef _INITCAMERAS_H_
#define _INITCAMERAS_H_

#include <iostream>
#include <optional>
#include <opencv2/highgui/highgui.hpp>

#include "Declarations.hpp"

void init_video_open(const int32_t device_number,
                    const int32_t frame_width,
                    const int32_t frame_height,
                    cv::VideoCapture& video_capture,
                    cv::Mat& frame);

void init_video_capture(const int32_t device_number,
                        const int32_t frame_width,
                        const int32_t frame_height,
                        cv::VideoCapture& video_capture,
                        cv::Mat& frame);
#endif