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

#ifndef _GENERALSEARCHFUNCTIONS_H_
#define _GENERALSEARCHFUNCTIONS_H_


#include <iostream>
#include <fstream>
#include <optional>
#include <opencv2/highgui/highgui.hpp>

#include "Declarations.hpp" // for Tag and Message structs

void update_tag(const cv::Point2f* detection,
                const cv::Point2f* cornerDetections,
                const boost::posix_time::ptime latest_frame,
                Tag* tag, std::ofstream& file_output);

float calc_velocity(const int old_x, const int old_y, 
                    const int new_x, const int new_y,
                    const boost::posix_time::ptime old_frame,
                    const boost::posix_time::ptime new_frame);

void add_detection_to_msg(const int id, uint64_t detectionTime_ms, 
                        const float room_x, const float room_y, const float room_z, 
                        const float theta, const size_t index, 
                        const int CAM_NAME, Message& buf);

inline bool tag_exists(const int x_center, 
                const int y_center) {
    return (0 < x_center && 0 < y_center);
}

void reset_tag(const int image_width, const int image_height, Tag *tag);

#endif