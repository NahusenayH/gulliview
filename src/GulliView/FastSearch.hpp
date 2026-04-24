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

#ifndef _FASTSEARCH_H_
#define _FASTSEARCH_H_

#include "FastSearchFunctions.hpp"
#include "GeneralSearchFunctions.hpp"
#include "LogTime.hpp"

void fast_search(const image_u8_t& im,
                const boost::posix_time::ptime latest_frame,
                const float v_max,
                const float a_max,
                const float alpha,
                const int min_search_dim,
                const int CAM_NAME,
                const float time_uncertainty,
                apriltag_detector_t* detector,
                zarray_t* detections,
                Tag* tags_start,
                bool& use_exhaustive_search,
                std::ofstream& file_output);

#endif