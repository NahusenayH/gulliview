#ifndef _FASTSEARCH_H_
#define _FASTSEARCH_H_

#include "FastSearchFunctions.hpp"
#include "GeneralSearchFunctions.hpp"

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
                std::ofstream& file_output
                );

void fast_search1(const image_u8_t& im,
                const boost::posix_time::ptime latest_frame,
                const float v_max,
                const float a_max,
                const float alpha,
                const int min_search_dim,
                const int CAM_NAME,
                const float time_uncertainty,
                apriltag_detector_t* detector,
                zarray_t* detections,
                Tag* tags_start);

void fast_search2(const image_u8_t& im,
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