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

#ifndef _FASTSEARCHFUNCTIONS_H_
#define _FASTSEARCHFUNCTIONS_H_

#include <fstream>
#include <optional>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "../TagFamily.h"

#include "Declarations.hpp"
#include "LogTime.hpp"

void set_search_area(const int32_t im_width,
                    const int32_t im_height,
                    const int min_search_dim,
                    const float min_travel,
                    const float max_travel,
                    const float alpha,
                    Tag& tag);

image_u8_t* get_partial_image(const image_u8_t& im, 
                                const DetectionArea& area);

inline float calc_displacement(const float velocity, 
                                const float time_s,
                                const float acceleration) {
    return velocity * time_s + 0.5 * acceleration * pow(time_s, 2);
}
            
void get_min_max_travel(const Tag* tag,
                        const float time_s,
                        const float v_max,
                        const float a_max,
                        float& min_travel, 
                        float& max_travel);

void partial_search(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector,
                    LogTime* parent_timer,
                    std::ofstream& file_output
                    );

#endif