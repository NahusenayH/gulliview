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
                std::ofstream& file_output
                ) {

    std::mutex detections_mutex; // Protect shared resource
    std::mutex log_mutex;        // Protect log messages

    boost::posix_time::ptime total_start_time = boost::posix_time::microsec_clock::universal_time();

    auto thread_task = [&](Tag* current_tag) {
        if (!tag_exists(current_tag->x, current_tag->y)) {
            return;
        }

        boost::posix_time::ptime search_start = boost::posix_time::microsec_clock::universal_time();

        auto elapsed = latest_frame - current_tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;

        float max_travel, min_travel;
        get_min_max_travel(current_tag, time_s, v_max, a_max, min_travel, max_travel);

        set_search_area(im.width, im.height, min_search_dim, min_travel, max_travel, alpha, *current_tag);

        {
            std::lock_guard<std::mutex> lock(log_mutex);
#if PRINT_DEBUG_MSG
            std::cout << "CAM#" << CAM_NAME << " using PART SEARCH "
                    << current_tag->area.x_length << "x" << current_tag->area.y_length
                    << "\n";
#endif
        }

        {
            std::lock_guard<std::mutex> lock(detections_mutex);
            partial_search(im, current_tag->area, detections, detector);
        }

        boost::posix_time::ptime search_end = boost::posix_time::microsec_clock::universal_time();

        uint32_t search_time = (search_end - search_start).total_microseconds();

        // {

        //     std::lock_guard<std::mutex> lock(log_mutex);

        //     int index = static_cast<int>(current_tag - tags_start);

        //     std::cout <<"CAM#"<<CAM_NAME<<": " <<"tag#"<< index <<": "<< "FAST SEARCH: search_time:" << search_time << " microseconds\n";

        // }

        uint32_t total_time = (search_end - total_start_time).total_microseconds();

        if (total_time > DEFAULT_LIMIT_MAX) {
#if PRINT_DEBUG_MSG
            std::cout << "CAM#" << CAM_NAME
                    << " exceeded " << DEFAULT_LIMIT_MAX / 1000 << "ms search time. Switching to exhaustive search.\n";
            use_exhaustive_search = true;
#endif
            return; // Terminate fast search early
        }

    };

    // Process tags with threads
    const size_t num_threads = std::min<size_t>(static_cast<size_t>(MAX_TAG_ID), std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        threads.emplace_back(thread_task, tag);
    }

    // Join threads
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

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
                Tag* tags_start) {

    int total_tag = 0;

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)){
            // tag was not detected during the previous exhaustive search
            continue;
        }

        total_tag ++;

        boost::posix_time::ptime search_start = boost::posix_time::microsec_clock::universal_time();


        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;  // time elapsed in seconds
        //Create part image to detect for each tag
        // image_u8_t* im_part = CreatePartImage(Areas[i], im);
        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                    min_travel, max_travel, alpha, *tag);
#if PRINT_DEBUG_MSG
        // modified 2025
        // cout <<"CAM#"<<CAM_NAME<<": " << "Using PART SEARCH " 
        //     << tag->area.x_length << "x" << tag->area.y_length 
        //     << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        std::cout <<"CAM#"<<CAM_NAME<<" " << "using PART SEARCH " 
        << tag->area.x_length << "x" << tag->area.y_length 
        << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
#endif
        partial_search(im, tag->area, detections, detector);


        boost::posix_time::ptime search_end = boost::posix_time::microsec_clock::universal_time();
        uint32_t search_time = (search_end - search_start).total_milliseconds();

        // modified 2025
        double st = boost::posix_time::milliseconds(search_time).total_microseconds();

        int index = static_cast<int>(tag - tags_start);

        std::cout <<"CAM#"<<CAM_NAME<<": " <<"tag#"<< index <<": "<< "FAST SEARCH: search_time: " << st << " microseconds\n";
    }
    std::cout << "total tag: " << total_tag << std::endl;
}

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
                std::ofstream& file_output) {

    int total_tag = 0;
    boost::posix_time::ptime total_start_time = boost::posix_time::microsec_clock::universal_time();

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)) {
            // Tag was not detected during the previous exhaustive search
            continue;
        }

        total_tag++;

        // ptime search_start = boost::posix_time::microsec_clock::universal_time();

        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;  // Time elapsed in seconds

        // file_output << "time s: " << time_s << endl;

        auto initial_start = std::chrono::high_resolution_clock::now();

        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                    min_travel, max_travel, alpha, *tag);

#if PRINT_DEBUG_MSG
        // file_output <<"CAM#"<<CAM_NAME<<": " << "Using PART SEARCH " 
        //     << tag->area.x_length << "x" << tag->area.y_length 
        //     << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        file_output <<"CAM#"<<CAM_NAME << " using PART SEARCH " 
        << tag->area.x_length << "x" << tag->area.y_length 
        << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
#endif

        auto initial_end = std::chrono::high_resolution_clock::now();
        auto initial_duration = std::chrono::duration_cast<std::chrono::microseconds>(initial_end - initial_start).count() / 1000.0;

        // file_output << "Execution time for the producer: " << std::fixed << std::setprecision(2) << initial_duration << " ms" << std::endl;

        auto partial_start = std::chrono::high_resolution_clock::now();

        float scaling_f = 0.125; // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit 

        // Calculate the size of the search area
        DetectionArea* area = &tag->area;
        double search_area_width = scaling_f * (area->x_end - area->x_start);
        double search_area_height = scaling_f * (area->y_end - area->y_start);
        double search_area_size = search_area_width * search_area_height;

        // file_output << "Search Area Size: " << search_area_size << endl;

        partial_search1(im, tag->area, detections, detector, total_start_time, file_output);

        auto partial_end = std::chrono::high_resolution_clock::now();
        auto partial_duration = std::chrono::duration_cast<std::chrono::microseconds>(partial_end - partial_start).count() / 1000.0;

        boost::posix_time::ptime search_end = boost::posix_time::microsec_clock::universal_time();
        // uint32_t search_time = (search_end - search_start).total_microseconds();

        int index = static_cast<int>(tag - tags_start);

#if PRINT_DEBUG_MSG
        // file_output <<"CAM#"<<CAM_NAME<<": " <<"tag#"<< index <<": "<< "FAST SEARCH: search_time: " << std::fixed << std::setprecision(2) << partial_duration << " milliseconds\n";
        file_output << "CAM#" << CAM_NAME << " tag#"<< index <<
        " FAST SEARCH time: " << std::fixed << std::setprecision(2) << partial_duration << " ms\n";
#endif
        uint32_t total_time = (search_end - total_start_time).total_microseconds();
        // cout << "fast_search2 time: " << total_time << endl;

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