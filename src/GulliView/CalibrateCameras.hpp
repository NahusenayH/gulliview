/*********************************************************************
* This file is distributed as part of the C++ port of the APRIL tags
* library. The code is licensed under GPLv2.
*
* Original author: Edwin Olson <ebolson@umich.edu>
* C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
* ----------------------- Modified ---------------------------------e
* Code modified for project in Vision Based Localization for
* Autonomous Vehicles at Chalmers University, Goteborg, Sweden
* Modification Authors:
* Copyright (c) 2013-2014 Andrew Soderberg-Rivkin <sandrew@student.chalmers.se>
* Copyright (c) 2013-2014 Sanjana Hangal <sanjana@student.chalmers.se>
* Copyright (c) 2014 Thomas Petig <petig@chalmers.se>
* Copyright (c) 2025 Emil Nylander <emilnyla@chalmers.se>
* Copyright (c) 2025 Elias Svensson <eliasve@chalmers.se>
********************************************************************/

#ifndef _CALIBRATECAMERAS_H_
#define _CALIBRATECAMERAS_H_

#include <optional>
#include <boost/asio.hpp>   //Included in declarations.hpp as well
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../AprilTypes.h"
#include "../TagFamily.h"

#include "TransformFrame.hpp"
#include "Declarations.hpp"

void setDestinationPoints(const int cam_name, at::Point* destination);
void init_undistortion_matrices(const cv::VideoCapture& video_capture,
                                const cv::Size& frame_size,
                                cv::Mat& map1,
                                cv::Mat& map2,
                                int camera_id);
void automated_calibration(const int32_t width,
                            const int32_t height,
                            at::Point* destination,
                            at::Point* source,
                            int32_t* camera,
                            cv::VideoCapture& video_capture,
                            cv::Mat &map1,
                            cv::Mat &map2, int camera_id);

#endif