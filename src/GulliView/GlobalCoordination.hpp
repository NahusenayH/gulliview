


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

cv::Mat matxvector(cv::Mat a, cv::Mat b);

cv::Mat addmatxvector(cv::Mat a, cv::Mat b);

cv::Mat average_mat(std::list<cv::Mat> mats);

#endif