#ifndef _FASTSEARCH_H_
#define _FASTSEARCH_H_

#include <fstream>
#include <optional>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "../TagFamily.h"

#include "Declarations.hpp"

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
                    apriltag_detector_t* detector);

void partial_search1(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector,
                    boost::posix_time::ptime total_start_time,
                    std::ofstream& file_output
                    );

#endif