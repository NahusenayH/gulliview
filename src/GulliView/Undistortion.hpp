#ifndef _UNDISTORTION_H_
#define _UNDISTORTION_H_

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

extern std::map<int, cv::Mat> camera_K_matrices;

extern std::map<int, cv::Mat> camera_global_distortion_coefficients;

cv::Mat undistort_points(cv::Mat points, int camera_id);


#endif