


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
void estimate_object_global_position(int camera_id, cv::Mat points, cv::Mat* global_position, cv::Mat* global_rotation);


#endif