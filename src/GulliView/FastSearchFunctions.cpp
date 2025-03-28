#include "FastSearchFunctions.hpp"

void set_search_area(const int32_t im_width,
                    const int32_t im_height,
                    const int min_search_dim,
                    const float min_travel,
                    const float max_travel,
                    const float alpha,
                    Tag& tag) {

    DetectionArea& area = tag.area;
    std::vector<float> xs{
        0.0f,
        max_travel * cosf(tag.theta),
        max_travel * cosf(tag.theta + alpha),
        max_travel * cosf(tag.theta - alpha),
        min_travel * cosf(tag.theta),
        min_travel * cosf(tag.theta + alpha),
        min_travel * cosf(tag.theta - alpha)
    };

    std::vector<float> ys{
        0.0f,
        max_travel * sinf(tag.theta),
        max_travel * sinf(tag.theta + alpha),
        max_travel * sinf(tag.theta - alpha),
        min_travel * sinf(tag.theta),
        min_travel * sinf(tag.theta + alpha),
        min_travel * sinf(tag.theta - alpha)
    };

    std::vector<float>::iterator x_min = std::min_element(xs.begin(), xs.end());
    std::vector<float>::iterator x_max = std::max_element(xs.begin(), xs.end());
    std::vector<float>::iterator y_min = std::min_element(ys.begin(), ys.end());
    std::vector<float>::iterator y_max = std::max_element(ys.begin(), ys.end());
    area.x_start = std::max(*x_min + tag.x - min_search_dim, 0.0f);
    area.y_start = std::max(*y_min + tag.y - min_search_dim, 0.0f);
    area.x_end = std::min(*x_max + tag.x + min_search_dim, float(im_width));
    area.y_end = std::min(*y_max + tag.y + min_search_dim, float(im_height));

    area.x_length = area.x_end - area.x_start;
    area.y_length = area.y_end - area.y_start;
}

image_u8_t* get_partial_image(const image_u8_t& im, const DetectionArea& area){
    image_u8_t* im_part = image_u8_create(area.x_length, area.y_length);
    int y_part = 0;
    for(int y = area.y_start; y < area.y_end; y++){
        uint8_t* dest = &im_part->buf[y_part * im_part->stride];
        uint8_t* src = &im.buf[y*im.stride + area.x_start];
        size_t size = area.x_length;
        memcpy(dest, src, size);
        y_part++;
    }
    return im_part;
}

void get_min_max_travel(const Tag* tag,
                        const float time_s,
                        const float v_max,
                        const float a_max,
                        float& min_travel, 
                        float& max_travel){

    max_travel = v_max * time_s;
    min_travel = -max_travel;

    if (tag->valid_velocity) {
        float tmp_travel = calc_displacement(tag->velocity, time_s, a_max);
        max_travel = std::min(tmp_travel, max_travel);
        tmp_travel = calc_displacement(tag->velocity, time_s, -a_max);
        min_travel = std::max(tmp_travel, min_travel);
    }
}

void partial_search(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector) {

    image_u8_t* im_part = get_partial_image(im, area);
    //detect tags in part image
    zarray_t *detection = apriltag_detector_detect(detector, im_part);

    if(zarray_size(detection) != 0){
        apriltag_detection_t *temp;
        zarray_get(detection, 0, &temp);
        zarray_add(detections, &temp);
    }
}

void partial_search1(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector,
                    boost::posix_time::ptime total_start_time,
                    std::ofstream& file_output
                    ) {

    auto start = std::chrono::high_resolution_clock::now();

    image_u8_t* im_part = get_partial_image(im, area);

    auto end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Get partial image time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif

    boost::posix_time::ptime search_end = boost::posix_time::microsec_clock::universal_time();

    uint32_t total_time = (search_end - total_start_time).total_microseconds();    
    if (total_time > 17000)
        return;

    start = std::chrono::high_resolution_clock::now();

    //detect tags in part image
    zarray_t *detection = apriltag_detector_detect(detector, im_part);

    end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Apriltag detector detect time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif

    search_end = boost::posix_time::microsec_clock::universal_time();

    total_time = (search_end - total_start_time).total_microseconds();    
    if (total_time > 17000)
        return;

    start = std::chrono::high_resolution_clock::now();


    if(zarray_size(detection) != 0){
        apriltag_detection_t *temp;
        zarray_get(detection, 0, &temp);
        zarray_add(detections, &temp);
    }

    end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Zarray time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif
}