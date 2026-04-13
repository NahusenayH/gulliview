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

#include "NiceThread.hpp"

zarray* exhaustive_search(image_u8_t& im, apriltag_detector_t* detector) {
    //detect tags
    return apriltag_detector_detect(detector, &im);
}

int nice_consume_frame(int camera_id, boost::interprocess::named_semaphore& sem, boost::interprocess::named_semaphore& sem_1, char* shared_memory, SharedData *ptr, cv::Mat frame, cv::Mat gray, int CAM_NAME, cv::Mat map1, cv::Mat map2, GulliViewOptions opts, std::string win, DebugLogger& nice_thread_logger) {

    std::ostringstream filename;
    filename << "output/camera_" << camera_id << "_output-nice.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    // pthread_t current_thread = pthread_self();

    // struct sched_param sched_param;
    // sched_param.sched_priority = 1; // Set priority to a nice value (higher = higher priority)

#if BINDING_CPU_CORES
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // Bind the thread to the corresponding core
    int thread_num = NICE_THREAD_NUM + camera_id % NICE_THREAD_COUNT * NICE_THREAD_COUNT / 4;
    CPU_SET(thread_num, &cpuset);

    // Set the CPU affinity of the thread
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
        return -1;
    }
#endif

    struct sched_param sched_param;
    int policy;

    // Get the scheduling policy and priority of the current thread
    if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    // std::cout << "Before setting, Thread priority: " << sched_param.sched_priority << std::endl;

    // Get the lowest priority of the SCHED_RR policy
    int min_priority = sched_get_priority_min(SCHED_RR);
    sched_param.sched_priority = min_priority;  // Set to lowest priority

    // Set the scheduling policy to SCHED_RR
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sched_param) != 0) {
        std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
        perror("Error details");
        return -1;
    }

    // Get the scheduling policy and priority of the current thread (confirm whether the setting is successful)
    if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    // file_output << "After setting, Thread priority: " << sched_param.sched_priority << std::endl;

    auto start = std::chrono::system_clock::now().time_since_epoch();
    std::queue<Message> messageQueue; // added 2024


    Tag tags[MAX_TAG_ID];

    TagFamily family(opts.family_str);
    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family.at_family);
    // apriltag_family_t *tf = tagStandard41h12_create(); // added 2025

    // apriltag_detector_add_family(detector, tf);

    detector->nthreads = 16;
    detector->quad_decimate = 1.0f; // 
    detector->quad_sigma = 0.6f; // Low-pass blur, negative values sharpen
    detector->refine_edges = 1; 

    // detector->quad_decimate = 2.0f; // Downsampling to increase speed
    // detector->quad_sigma = 0.0f; // Turn off blurring to reduce calculations
    // detector->refine_edges = 0; // Turn off edge refinement

    uint32_t seq = 0;
    int loop_count = 0;

    while (true) {

        // Get the scheduling policy and priority of the current thread
        if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            break;
        }



#if BINDING_CPU_CORES
        // Get the CPU affinity of the current thread
        if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to get CPU affinity for thread" << std::endl;
            break;
        }

        // Find the core on which the current thread is running
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &cpuset)) {
#if ENABLE_NICE_LOGS
                file_output << "Core number: " << i << std::endl;
#endif
                break;
            }
        }
#endif

        // Start measurement
        auto loop_start = std::chrono::high_resolution_clock::now();

        float avg_time_gap = -1;

        auto detectionTime = std::chrono::system_clock::now().time_since_epoch();
        uint64_t detectionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(detectionTime).count();


#if PRODUCE_FRAME_MODE == 1 || PRODUCE_FRAME_MODE == 3

        // Start measurement
        LogTime consumer_wait_timer;
        
        // Wait for the producer to produce a frame
        while (nice_consumer_counter[camera_id].load() == producer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

#if ENABLE_NICE_LOGS
        consumer_wait_timer.stop_ms("Nice thread waiting for producer", file_output);
#endif

        // Atomically load the published index with memory synchronization
        int index = producer_counter[camera_id].load(std::memory_order_acquire);

        nice_consumer_counter[camera_id] = index;

        FrameData& frame_data = buffer[camera_id][index];
        cv::Mat frame = frame_data.frame;
        LogTime frametime = frame_data.frametime;

        // Check if the frame is empty
        // if (frame.empty()) {
        //     std::cout << "No frame recieved in nice thread, camera " << camera_id << std::endl;
        //     break;
        // }

        //handle empty frames 
        if (frame.empty()) {
            thread_local std::chrono::steady_clock::time_point last_log_time = std::chrono::steady_clock::time_point::min();
            auto now = std::chrono::steady_clock::now();
            if (now - last_log_time > std::chrono::seconds(1)) {
                std::cout << "No frame recieved in nice thread, camera " << camera_id << std::endl;
                last_log_time = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        boost::posix_time::ptime transform_start = boost::posix_time::microsec_clock::universal_time();
        bool frame_captured = transform_frame(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            std::cout << "no frame captured (nice), exiting. Camera " << camera_id << std::endl;
            return -1;
        }

        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

        // image_u8_t im = {
        //     .width = frame.cols,
        //     .height = frame.rows,
        //     .stride = frame.cols,
        //     .buf = frame.data
        // };

        boost::posix_time::ptime transform_end = boost::posix_time::microsec_clock::universal_time();
        uint32_t transform_time = (transform_end - transform_start).total_milliseconds();
        //	std::cout << transform_time << std::endl;

#elif PRODUCE_FRAME_MODE == 2

        ptime latest_frame = boost::posix_time::microsec_clock::universal_time();

        BufferData data = lock_buffer->consume_nice(); // Read from buffer
        frame = data.frame;
        // gray = data.gray;

        bool frame_captured = transform_frame1(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            std::cout << "no frame captured, exiting" << std::endl;
            return -1;
        }


        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

#else
    #error "Unsupported mode"
#endif
#if PRINT_DEBUG_MSG
        nice_thread_logger.log_operation(DebugLogger::TRANSFORM_TIME, transform_time, nice_consumer_counter[camera_id].load());
# endif
        zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*)); //2023: from FastSearch-code

        //Use exhaustive search

        loop_count = 1;
        //Clear previous coordinates of all tags.
        for (Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++) {
            reset_tag(im.width, im.height, tag);
        }

        auto search_start = std::chrono::high_resolution_clock::now();

        boost::posix_time::ptime latest_frame = boost::posix_time::microsec_clock::universal_time();
            
        detections = exhaustive_search(im, detector);

#if ENABLE_NICE_LOGS
        file_output << detections << std::endl;
#endif

        auto search_end = std::chrono::high_resolution_clock::now();   // End measurement
        double search_time = std::chrono::duration_cast<std::chrono::microseconds>(search_end - search_start).count();

        DetectionData detection_data;


#if PRINT_DEBUG_MSG

        // modified 2025
       file_output <<"CAM#"<<CAM_NAME<< " GLOBAL SEARCH time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";

        nice_thread_logger.log_operation(DebugLogger::GLOBAL_SEARCH_TIME, search_time, nice_consumer_counter[camera_id].load());

#endif
        if (zarray_size(detections) != 0) {
            // Get time of frame/detection----------------

            Message buf {
                htobe32(1) /* type */ ,
                htobe32(2) /* subtype */,
                htobe32(seq) /* seq */,
                // added 2024
                htobe64(detectionTime_ms), /* detection timestamp */
                htobe64(avg_time_gap) /* avg time gap */
                // htobe64(msecs) /* time_msec */   // commented out 2024
            };
            // Camera coordinates for tag center.
            std::vector<at::Point> camera_detections(zarray_size(detections)); 
            // Camera coordinates for tag corners.
            std::vector<at::Point> camera_corner_detections(2*zarray_size(detections)); 

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;
                camera_detections[i] = at::Point(dd->c[0], dd->c[1]);
                camera_corner_detections[2*i] = at::Point(dd->p[0][0], dd->p[0][1]);
                camera_corner_detections[2*i+1] = at::Point(dd->p[3][0], dd->p[3][1]);
                // Aron: Adjust for part image coordinates?
                camera_detections[i].x += tag->area.x_start;
                camera_detections[i].y += tag->area.y_start;      
            }
            // Room coordinates for tag center.
            std::vector<at::Point> room_detections(zarray_size(detections)); 
            // Room coordinates for tag corner.
            std::vector<at::Point> room_corner_detections(2*zarray_size(detections)); 

            
            static boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));

            buf.cam_id = htobe32(CAM_NAME);
            int n_detections = zarray_size(detections);
            std::vector<cv::Point2f> points;

            for (int i = 0; i < n_detections; i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;

                int x_area_start = tag->area.x_start;
                int y_area_start = tag->area.y_start;
                cv::Mat distorted_points = (cv::Mat_<double>(4,2) << dd->p[3][0] + x_area_start, dd->p[3][1] + y_area_start,
                                                                     dd->p[2][0] + x_area_start, dd->p[2][1] + y_area_start,
                                                                     dd->p[1][0] + x_area_start, dd->p[1][1] + y_area_start,
                                                                     dd->p[0][0] + x_area_start, dd->p[0][1] + y_area_start);
                cv::Mat undistorted_points = undistort_points(distorted_points, camera_id);

                cv::Mat world_position, world_rotation;
                world_position = estimate_object_global_position(camera_id, undistorted_points, &world_position, &world_rotation, frame);

                cv::Point2f* cornerDetection = 2*i + room_corner_detections.data();
                cv::Point2f* detection = i + camera_detections.data();
                update_tag(detection, cornerDetection, latest_frame, tag, file_output);
                detection = i + room_detections.data();

                float global_x_m = float (world_position.at<double>(0)); // gets meter coordinates of x axis
                float global_y_m = float (world_position.at<double>(1)); // gets meter coordinates of y axis
                float global_z_m = float (world_position.at<double>(2)); // gets meter coordinates of z axis

                add_detection_to_msg(dd->id, frametime.timestamp(), global_x_m, global_y_m, global_z_m, //tag->x, tag->y
                    tag->theta, i, CAM_NAME, buf);   // added 2024, "detectionTime_ms" added

                detection_data.tags[dd->id].found = true;
                // detection_data.tags[dd->id].area = tag->area;
                detection_data.tags[dd->id].camera_coords = DetectionData::CameraCoordinates{tag->x, tag->y, tag->theta};

            // First create an apriltag_detection_info_t struct using your known parameters.
                apriltag_detection_info_t info;
                info.det = dd;
                info.tagsize = opts.tag_size;

#if PRINT_DEBUG_MSG
                file_output << "Tag size of opts: " << opts.tag_size << std::endl;
#endif

                // Define the parameter table corresponding to the device
                struct DeviceInfo {
                    double fx_2k, fx_4k;
                    double fy_2k, fy_4k;
                    double cx_2k, cx_4k;
                    double cy_2k, cy_4k;
                };

                static const DeviceInfo device_params[] = {
                    {1135.208474, 2270.416948, 1133.531325, 2267.062650, 998.932805, 1997.865610, 530.084624, 1060.169248}, // Device 1
                    {1146.495853, 2290.512606, 1146.011083, 2283.316632, 953.976032, 1923.229236, 571.868765, 1028.981542}, // Device 2
                    {1145.256303, 2292.991706, 1141.658316, 2292.022166, 961.614618, 1907.952064, 514.490771, 1143.737530}, // Device 3
                    {1148.845354, 2297.690708, 1136.968276, 2273.936552, 987.148727, 1974.297454, 523.721356, 1047.443252}  // Device 4
                };

                if (opts.device_num >= 1 && opts.device_num <= 4) {
                    const DeviceInfo& params = device_params[opts.device_num - 1];

                    if (opts.frame_width == 3840 && opts.frame_height == 2160) { // 4K
                        info.fx = params.fx_4k;
                        info.fy = params.fy_4k;
                        info.cx = params.cx_4k;
                        info.cy = params.cy_4k;
                    } else if (opts.frame_width == 1920 && opts.frame_height == 1080) { // 2K
                        info.fx = params.fx_2k;
                        info.fy = params.fy_2k;
                        info.cx = params.cx_2k;
                        info.cy = params.cy_2k;
                    } else {
                        // Default value or error handling
                        throw std::invalid_argument("Unsupported resolution");
                    }
                } else {
                    throw std::invalid_argument("Unsupported device number");
                }

            }

            buf.length = htobe32(n_detections);

            detection_data.storeMessage(buf);
#if ENABLE_NICE_LOGS
            frametime.stop_ms("Latency nice", file_output);
#endif

        }

        std::copy(std::begin(tags), std::end(tags), detection_data.tag_data);

        auto start = std::chrono::high_resolution_clock::now();

        unsigned int search_next = (search_producer_counter[camera_id].load() + 1) % BUFFER_SIZE;

        while(search_next == search_consumer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        search_buffer[camera_id][search_next] = detection_data;
        search_producer_counter[camera_id] = search_next;

        // End measurement
        auto end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

#if PRINT_DEBUG_MSG
        // printing time
        file_output << "Nice thread waiting for producer: " << std::fixed << std::setprecision(2) << duration / 1000.0 << " ms" << std::endl;


        nice_thread_logger.log_operation(DebugLogger::PROCESS_TIME, duration, nice_consumer_counter[camera_id].load());
#endif
        //If there are tags missing, use exhaustive search next time

        if (sig_stop) {
            break;
        }
        apriltag_detections_destroy(detections);

        loop_count++;

            // End measurement
    auto loop_end = std::chrono::high_resolution_clock::now();

    // Calculation time (in microseconds)
    auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();

#if PRINT_DEBUG_MSG
    // printing time
    file_output << "Execution time for one loop: " << std::fixed << std::setprecision(2) << loop_duration / 1000.0 << " ms" << std::endl;


    nice_thread_logger.log_operation(DebugLogger::LOOP_TIME, loop_duration, nice_consumer_counter[camera_id].load());

    nice_thread_logger.write_to_file_if_needed(loop_duration, "nice_thread", filename.str());
#endif

    }

    apriltag_detector_destroy(detector);
    file_output.close();

    std::cout << "Camera " << camera_id << " nice exiting" << std::endl;

    return 0;
}
