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


#ifndef _GLOBALCOORDINATION_H_
#define _GLOBALCOORDINATION_H_

#include <iostream>
#include "opencv2/core/cvstd.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <string>
#include <numeric>
#include <list>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h" // added 2025;
#include "apriltag/common/image_u8.h" // added 2025;

#include "Undistortion.hpp" // for K and distortion_coeffs matrices

cv::Mat matxvector(cv::Mat a, cv::Mat b);

cv::Mat addmatxvector(cv::Mat a, cv::Mat b);

cv::Mat average_mat(std::list<cv::Mat> mats);

// Input: camera_id, obj2cam_rvec, obj2cam_tvec
// Output: global position, global_rotation
cv::Mat estimate_object_global_position(int camera_id, cv::Mat points, cv::Mat* global_position, cv::Mat* global_rotation, cv::Mat image);

cv::Mat global_to_pixel(int camera_id, cv::Mat global_position, cv::Mat image);

#endif