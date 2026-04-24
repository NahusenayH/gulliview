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

#include "GUI.hpp"

void update_gui(const zarray_t* detections, Tag* tags_start, cv::Mat& frame, float avg_hz, std::ofstream& file_output) {

    float scaling_f = GUI_SCALE;

    std::ostringstream ss;
    ss << avg_hz;

    cv::resize(frame, frame, cv::Size(), scaling_f, scaling_f, cv::INTER_LINEAR);

    putText(frame, ss.str(),
        cv::Point(30, 30),
        cv::FONT_HERSHEY_PLAIN,
        2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    if (zarray_size(detections) == 0) {
        std::string idToText = "---Nothing Detected---";
        putText(frame, idToText,
                cv::Point(30, 30),
                cv::FONT_HERSHEY_PLAIN,
                1.5, cv::Scalar(180, 250, 0), 1, cv::LINE_AA);

        return;
    }
    // Get time of frame/detection----------------
    //show = family.superimposeDetections(frame, detections); //-- Used to actually
    //superimpose tag image in video
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *dd;
        zarray_get(detections, i, &dd);
        Tag* tag = tags_start + dd->id;
        DetectionArea* area = &tag->area;
        // Draw green square around the search area
        cv::rectangle(frame,
                    cv::Point(scaling_f*area->x_start, scaling_f*area->y_start),
                    cv::Point(scaling_f*area->x_end, scaling_f*area->y_end),
                    cv::Scalar(0, 255, 0));
        // Print out Tag ID in center of Tag
        putText(frame, std::to_string(dd->id), cv::Point(scaling_f*tag->x, scaling_f*tag->y),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 250, 0), 2,
                cv::LINE_AA);

    // Calculate the size of the AprilTag in pixels
        double apriltag_size = 0.0;
        for (int j = 0; j < 4; j++) {
            int next = (j + 1) % 4;
            double dx = dd->p[next][0] - dd->p[j][0];
            double dy = dd->p[next][1] - dd->p[j][1];
            apriltag_size += sqrt(dx * dx + dy * dy); // Sum edge lengths
        }
        apriltag_size /= 4.0; // Average size of edges

        // Calculate the size of the search area
        double search_area_width = scaling_f * (area->x_end - area->x_start);
        double search_area_height = scaling_f * (area->y_end - area->y_start);
        double search_area_size = search_area_width * search_area_height;

        // Calculate the ratio of search area size to AprilTag size
        // double ratio = search_area_size / apriltag_size;

        // Print out the ratio and related information

#if PRINT_DEBUG_MSG
        file_output << "Tag ID: " << dd->id
                << ", AprilTag Size: " << apriltag_size
                << ", Search Area Size: " << search_area_size
                << ", Ratio: " << ratio << std::endl;
#endif

    }
}

void update_exhaustive_gui(DetectionData detection_data, Tag* tags_start, cv::Mat& frame, float avg_hz) {

    float scaling_f = GUI_SCALE;

    std:: ostringstream ss;
    ss << avg_hz;

    cv::resize(frame, frame, cv::Size(), scaling_f, scaling_f, cv::INTER_LINEAR);

    putText(frame, ss.str(),
        cv::Point(30, 30),
        cv::FONT_HERSHEY_PLAIN,
        2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    // Get time of frame/detection----------------
    //show = family.superimposeDetections(frame, detections); //-- Used to actually
    //superimpose tag image in video

    bool found = false;

    for (int i = 0; i < 10; i++) 
        if (detection_data.tags[i].found){
            found = true;

            // Tag* tag = tags_start + i;
            DetectionArea* area = &detection_data.tag_data[i].area;
            // Draw green square around the search area
            cv::rectangle(frame,
                        cv::Point(scaling_f*area->x_start, scaling_f*area->y_start),
                        cv::Point(scaling_f*area->x_end, scaling_f*area->y_end),
                        cv::Scalar(0, 255, 0));
            // Print out Tag ID in center of Tag
            putText(frame, std::to_string(i), cv::Point(scaling_f*detection_data.tags[i].camera_coords->x, scaling_f*detection_data.tags[i].camera_coords->y),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 250, 0), 2,
                    cv::LINE_AA);
        }

    if (!found) {
        std::string idToText = "---Nothing Detected---";
        putText(frame, idToText,
                cv::Point(30, 30),
                cv::FONT_HERSHEY_PLAIN,
                1.5, cv::Scalar(180, 250, 0), 1, cv::LINE_AA);

        return;
    }
}
