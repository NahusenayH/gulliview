#ifndef _CALIBRATECAMERAS_H_
#define _CALIBRATECAMERAS_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/asio.hpp>   //Included in declarations.hpp as well
#include <optional>

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