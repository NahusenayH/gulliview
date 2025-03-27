#ifndef _GENERALSEARCHFUNCTIONS_H_
#define _GENERALSEARCHFUNCTIONS_H_

#include <unordered_map> // added 2025;
#include "opencv2/core/cvstd.hpp"
#include "pthread.h"
#include <optional>

#include <fstream> // added 2025;
#include <eigen3/Eigen/Dense> // added 2025;
#include <eigen3/Eigen/Geometry> // added 2025;

#include <ctime>
#include <iostream>
#include <cstdio>
#include <getopt.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <csignal>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <thread>     // added 2024
#include <queue> // added 2024
#include <vector> // added 2024
#include <tuple> // added 2024
#include <regex> // added 2024
#include <fcntl.h> // added 2024
#include <sys/stat.h> // added 2024
#include <sys/mman.h> // added 2024
#include <sstream> // added 2024
#include <boost/interprocess/sync/named_semaphore.hpp> // added 2024
#include <boost/interprocess/file_mapping.hpp> // added 2024
#include <boost/interprocess/mapped_region.hpp> // added 2024

#include <sstream>
#include <thread>

#include "Declarations.hpp" // for Tag and Message structs

; // DO NOT COMMENT THIS SEMICOLON! CODE WILL NOT WORK
void update_tag(const cv::Point2f* detection,
                const cv::Point2f* cornerDetections,
                const boost::posix_time::ptime latest_frame,
                Tag* tag, std::ofstream& file_output);

float calc_velocity(const int old_x, const int old_y, 
                    const int new_x, const int new_y,
                    const boost::posix_time::ptime old_frame,
                    const boost::posix_time::ptime new_frame);

void add_detection_to_msg(const int id, uint64_t detectionTime_ms, const float room_x, const float room_y, 
                        const float theta, const size_t index, 
                        const int CAM_NAME, Message& buf);

inline bool tag_exists(const int x_center, 
                const int y_center) {
    return (0 < x_center && 0 < y_center);
}

void reset_tag(const int image_width, const int image_height, Tag *tag);

#endif