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

#include "FastSearch.hpp"

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
                std::ofstream& file_output) {

    int total_tag = 0;
    boost::posix_time::ptime total_start_time = boost::posix_time::microsec_clock::universal_time();

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)) {
            // Tag was not detected during the previous exhaustive search
            continue;
        }

        total_tag++;

        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;

        auto initial_start = std::chrono::high_resolution_clock::now();

        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                    min_travel, max_travel, alpha, *tag);

        auto initial_end = std::chrono::high_resolution_clock::now();
        auto initial_duration = std::chrono::duration_cast<std::chrono::microseconds>(initial_end - initial_start).count() / 1000.0;


        auto partial_start = std::chrono::high_resolution_clock::now();

        float scaling_f = 0.125; // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit 

        // Calculate the size of the search area
        DetectionArea* area = &tag->area;
        double search_area_width = scaling_f * (area->x_end - area->x_start);
        double search_area_height = scaling_f * (area->y_end - area->y_start);
        double search_area_size = search_area_width * search_area_height;

        partial_search(im, tag->area, detections, detector, total_start_time, file_output);

        auto partial_end = std::chrono::high_resolution_clock::now();
        auto partial_duration = std::chrono::duration_cast<std::chrono::microseconds>(partial_end - partial_start).count() / 1000.0;

        boost::posix_time::ptime search_end = boost::posix_time::microsec_clock::universal_time();

        int index = static_cast<int>(tag - tags_start);

#if PRINT_DEBUG_MSG
        file_output << "CAM#" << CAM_NAME << " tag#"<< index <<
        " FAST SEARCH time: " << std::fixed << std::setprecision(2) << partial_duration << " ms\n";
#endif
        uint32_t total_time = (search_end - total_start_time).total_microseconds();

        if (total_time > DEFAULT_LIMIT_MAX) {
#if PRINT_DEBUG_MSG
            file_output << "CAM#" << CAM_NAME << " tag#"<< index <<
                " exceeded " << DEFAULT_LIMIT_MAX / 1000 << "ms search time. Switching to exhaustive search.\n";
#endif
            use_exhaustive_search = true;
            break; // Terminate fast search early
        }
    }
}