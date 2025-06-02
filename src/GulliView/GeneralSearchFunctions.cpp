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

#include "GeneralSearchFunctions.hpp"

void update_tag(const cv::Point2f* detection,
                const cv::Point2f* cornerDetections,
                const boost::posix_time::ptime latest_frame,
                Tag* tag, std::ofstream& file_output) {

    if (tag_exists(tag->x, tag->y)) {
        tag->velocity = calc_velocity(tag->x, tag->y, 
                        detection->x, detection->y,
                        tag->latest_detection, latest_frame);
        tag->valid_velocity = true;

#if PRINT_DEBUG_MSG
        file_output << "tag->x: " << tag->x << " tag->y: " << tag->y << " detection->x: " << detection->x << " detection->y: " << detection->y << std::endl;
#endif
    }

    tag->x = detection->x;
    tag->y = detection->y;
    tag->latest_detection = latest_frame;
    tag->is_detected = true;
    float x0 = (cornerDetections + 1)->x;
    float y0 = (cornerDetections + 1)->y;
    float x1 = cornerDetections->x;
    float y1 = cornerDetections->y;
    tag->theta = atan2(y1 - y0, x1 - x0);
}


float calc_velocity(const int old_x, const int old_y, 
                    const int new_x, const int new_y,
                    const boost::posix_time::ptime old_frame,
                    const boost::posix_time::ptime new_frame) {
    float dy = new_y - old_y;
    float dx = new_x - old_x;
    float diag = sqrt(powf(dx, 2) + powf(dy, 2));
    auto elapsed = new_frame - old_frame;
    auto dt = (elapsed).total_microseconds();
    return diag / (dt / 1e6f);
}


// modifeid 2024, "detectoinTime_ms" added
void add_detection_to_msg(const int id, uint64_t detectionTime_ms, 
                        const float room_x, const float room_y, const float room_z, 
                        const float theta, const size_t index, 
                        const int CAM_NAME, Message& buf) {
    int32_t x_coord = (int32_t) (room_x * 1000.0); // ELIAS2025 uncommented this
    int32_t y_coord = (int32_t) (room_y * 1000.0); // ELIAS2025 uncommented this
    int32_t z_coord = (int32_t) (room_z * 1000.0); // EMIL2025 added this

    union {
        float        f;
        unsigned int i;
        } angle;

    union {
        float        f;
        unsigned int i;
    } speed_f;
    
    float speed = 0.25f; // I HAVE SET THIS TO AN ARBITRARY VALUE SINCE REMOVING THE "safeSpeed" FUNCTION. 
                            // IT IS SENT TO SOCKET BUT NOT USED LATER.    // Convert theta to big endian angle
    angle.f = theta;
    angle.i = htobe32(angle.i);
    // Convert speed to big endian speed
    speed_f.f = speed;
    speed_f.i = htobe32(speed_f.i);

    buf.detections[index] = {      
        htobe32(id),  /* id */
        htobe64(detectionTime_ms),   /* added 2024*/
        htobe32(x_coord), /* x */
        htobe32(y_coord), /* y */
        htobe32(z_coord), /* z */
        angle.f,          /* angle theta */
        speed_f.f,        /* speed */
        htobe32(CAM_NAME) /* camera_id */
    };

// #if ENABLE_ANY_LOGS
//     std::cout << "[*] Camera: " << CAM_NAME << " Tag: " << id 
//         << " X: " << room_x << " Y: " << room_y << " Z: " << room_z << " Theta: " 
//         << theta << " Speed: " << speed << " Time: " 
//         << detectionTime_ms << std::endl;
// #endif
}



void reset_tag(const int image_width, const int image_height, Tag *tag) {
    tag->x = 0;
    tag->y = 0;
    tag->valid_velocity = false;
    tag->velocity = 0;
    tag->area.x_start = 0;
    tag->area.y_start = 0;
    tag->area.x_end = image_width;
    tag->area.y_end = image_height;
    tag->area.x_length = image_width;
    tag->area.y_length = image_height;
}
