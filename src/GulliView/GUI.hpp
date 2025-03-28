#ifndef _GUI_H_
#define _GUI_H_

#include <fstream>
#include <optional>
#include <opencv2/imgproc/imgproc.hpp>

#include "apriltag/apriltag.h"

#include "Declarations.hpp"

void update_gui(const zarray_t* detections, 
                Tag* tags_start, 
                cv::Mat& frame, 
                float avg_hz, 
                std::ofstream& file_output);

void update_exhaustive_gui(DetectionData detection_data, 
                        Tag* tags_start, 
                        cv::Mat& frame, 
                        float avg_hz);

#endif