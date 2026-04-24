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

    LogTime timer_total;

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)) {
            // Tag was not detected during the previous exhaustive search
            continue;
        }


        total_tag++;

        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;

        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                    min_travel, max_travel, alpha, *tag);

        LogTime partial_search_timer;

        partial_search(im, tag->area, detections, detector, &timer_total, file_output);

#if ENABLE_FAST_LOGS
        int index = static_cast<int>(tag - tags_start);
        file_output << "Part search time: tag#=" << index << ", time=" << partial_search_timer.stop_ms() << " ms" << std::endl;
        timer_total.stop_ms("Fast search time", file_output);
#endif

        if (timer_total.stop_us() > DEFAULT_LIMIT_MAX) {
#if ENABLE_FAST_LOGS
            file_output << "Exceeded fast search time: limit=" << DEFAULT_LIMIT_MAX / 1000 << " ms, tagID=" << index << std::endl;
#endif
            use_exhaustive_search = true;
            break; // Terminate fast search early
        }
    }
}