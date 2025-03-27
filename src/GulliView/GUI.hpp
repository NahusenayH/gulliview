#ifndef _GUI_H_
#define _GUI_H_

#include <cstring>
#include <cstdlib>
#include <string>
#include "apriltag/apriltag.h"
#include "opencv2/core/cvstd.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <fstream>
#include <cmath>

#include "Declarations.hpp"

void update_gui(const zarray_t* detections, Tag* tags_start, cv::Mat& frame, float avg_hz, std::ofstream& file_output);

void update_exhaustive_gui(DetectionData detection_data, Tag* tags_start, cv::Mat& frame, float avg_hz);

#endif