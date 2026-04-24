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

#include "TransformFrame.hpp"

bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2,
    std::ofstream& file_output // added 2025
    ) {
    // TODO save timestamp (maybe return the timestamp instead of bool)
    // cv::Mat undistorted_frame;

    if (frame.empty()) {
        std::cout << "no frame to transform, exiting" << std::endl;
        return false;
    }

//     LogTime remap_timer;
//     cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);
// #if ENABLE_ANY_LOGS
//     remap_timer.stop_ms("Remap", file_output);
// #endif

    LogTime color_timer;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
#if ENABLE_ANY_LOGS
    color_timer.stop_ms("Transform color", file_output);
#endif

    return !frame.empty();
}

bool transform_frame(cv::Mat& frame,
                    cv::Mat& gray,
                    cv::Mat& map1,
                    cv::Mat& map2) {
    // TODO save timestamp (maybe return the timestamp instead of bool)
    // cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);

    // Resize to 1080p if frame is 4K
    // if (frame.cols == 3840 && frame.rows == 2160) {
    //     cv::resize(frame, frame, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
    // }
    
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    return !frame.empty();
}