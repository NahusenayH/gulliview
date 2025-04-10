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

#ifndef _TRANSFORMFRAME_H_
#define _TRANSFORMFRAME_H_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "Declarations.hpp"
#include "LogTime.hpp"

bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2,
    std::ofstream& file_output // added 2025
    );
bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2);

#endif