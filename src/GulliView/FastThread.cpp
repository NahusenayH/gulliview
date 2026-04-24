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

#include "FastThread.hpp"
#include "GlobalCoordination.hpp"
#include "LogTime.hpp"
#include "Undistortion.hpp"

std::map<int, Eigen::Matrix4d> camera_to_world_matrices = {
    {0, (Eigen::Matrix4d() << 2270.416948, 0.0, 1997.865610, 0.0,
                            0.0, 2267.062650, 1060.169248, 0.0,
                            0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0).finished()},
    {1, (Eigen::Matrix4d() << 2290.512606, 0.0, 1923.229236, 0.0,
                            0.0, 2283.316632, 1028.981542, 0.0,
                            0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0).finished()},
    {2, (Eigen::Matrix4d() << 2292.991706, 0.0, 1907.952064, 0.0,
                            0.0, 2292.022166, 1143.737530, 0.0,
                            0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0).finished()},
    {3, (Eigen::Matrix4d() << 2297.690708, 0.0, 1974.297454, 0.0,
                            0.0, 2273.936552, 1047.443252, 0.0,
                            0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0).finished()},
};

float get_uncertainty() {
    float t_min;
    float t_max;
    switch (nines) {
    case 1:
        t_min = 0.031;
        t_max = 0.094;
        break;
    case 2:
        t_min = 0.031;
        t_max = 0.097;
        break;
    case 3:
        t_min = 0.028;
        t_max = 0.100;
        break;
    case 4:
        t_min = 0.023;
        t_max = 0.100;
        break;
    case 5:
        t_min = 0.015; //0.021;
        t_max = 0.200; //0.875;
        break;
    case 6:
        t_min = 0.015; //0.0165;
        t_max = 0.200; //8.8285;
        break;
    default:
        t_min = 0;
        t_max = 0;
    }
    return t_max - t_min;
}

int fast_consume_frame(int camera_id,
                        int thread_id, 
                        boost::interprocess::named_semaphore& sem, 
                        boost::interprocess::named_semaphore& sem_1, 
                        char* shared_memory, 
                        SharedData *ptr, 
                        cv::Mat frame, 
                        cv::Mat gray, 
                        int CAM_NAME, 
                        cv::Mat map1, 
                        cv::Mat map2, 
                        GulliViewOptions opts, 
                        std::string win, 
                        DebugLogger& fast_thread_logger) {

#if BINDING_CPU_CORES
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // bind the thread to the corresponding core
    int thread_num = FAST_THREAD_NUM + thread_id % FAST_THREAD_COUNT * FAST_THREAD_COUNT / 4;
    CPU_SET(thread_num, &cpuset);

    // set the CPU affinity of the thread
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
        return -1;
    }
#endif
    
    struct sched_param param;
    int policy;

    // set the priority to the highest (the maximum priority of SCHED_RR)
    param.sched_priority = sched_get_priority_max(SCHED_RR);

    // set the scheduling policy to SCHED_RR
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
        perror("Error details");
        return -1;
    }

    // get the scheduling policy and priority of the current thread
    if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    std::ostringstream filename;
    filename << "output/camera_" << camera_id << "_output-fast.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    Tag tags[MAX_TAG_ID];
    Tag previous_tags[MAX_TAG_ID];
    TagFamily family(opts.family_str);
    apriltag_detector_t* detector = apriltag_detector_create();

    apriltag_detector_add_family(detector, family.at_family);

    detector->nthreads = 16;
    detector->quad_decimate = 1.0f;
    detector->quad_sigma = 0.6f; // Low-pass blur, negative values sharpen
    detector->refine_edges = 1; 

    // detector->quad_decimate = 2.0f; // Downsampling to increase speed
    // detector->quad_sigma = 0.0f; // Turn off blurring to reduce calculations
    // detector->refine_edges = 0; // Turn off edge refinement

    const float time_uncertainty = get_uncertainty();
    bool use_exhaustive_search = true;
    const float pixels_per_meter = opts.frame_width / ROOM_WIDTH_METER;
    const float tag_diag_m = 0.428f;
    float a_max = opts.acceleration_max * pixels_per_meter;
    float v_max = opts.velocity_max * pixels_per_meter;
    const int min_search_dim = ceil(0.5 * tag_diag_m * pixels_per_meter);   // min dimension find a tag at any angle. 92 is diagonal of 36h11 tag, converted to pixels.
                                    // halved to account for search area creation
    float alpha = M_PI / 12; // modified 2025


    // cout << a_max << " " << v_max << " " << min_search_dim << " " << alpha << endl;

    uint32_t seq = 0;
    int global_search_counter = 0;
    int loop_count = 0;
    int hz_counter = 0;
    int tot_hz = 0;
    int avg_hz = 0;
    int sum_hz = 0;

    AngleTracker angle_tracker;
    AccelerationTracker acceleration_tracker;

    // Mapping of camera and buffer
    CyclicBuffer* produce_buffers[2] = {nullptr, nullptr};
    CyclicBuffer* consume_buffers[2] = {nullptr, nullptr};

    if (camera_id == 0) {
        produce_buffers[0] = &buffer_01;
        consume_buffers[0] = &buffer_10;
    } else if (camera_id == 1) {
        produce_buffers[0] = &buffer_10;
        produce_buffers[1] = &buffer_12;
        consume_buffers[0] = &buffer_01;
        consume_buffers[1] = &buffer_21;
    } else if (camera_id == 2) {
        produce_buffers[0] = &buffer_21;
        produce_buffers[1] = &buffer_23;
        consume_buffers[0] = &buffer_12;
        consume_buffers[1] = &buffer_32;
    } else if (camera_id == 3) {
        produce_buffers[0] = &buffer_32;
        consume_buffers[0] = &buffer_23;
    }

    for (int trial = 1; trial <= 1; trial++) {

    int total_loop_count = 0;

    while (true) {

        // Get the scheduling policy and priority of the current thread
        if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            break;
        }

#if BINDING_CPU_CORES
        // Get the CPU affinity of the current thread
        if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to get CPU affinity for thread" << std::endl;
            break;
        }

        // Find the core where the current thread is running
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &cpuset)) {
#if ENABLE_FAST_LOGS
                file_output << "Core number: " << i << std::endl;
#endif
                break;
            }
        }
#endif
        total_loop_count++;

        // Start measurement
        LogTime timer;
        // auto loop_start = std::chrono::high_resolution_clock::now();

        float avg_time_gap = -1;
        // auto while_start = std::chrono::system_clock::now().time_since_epoch();

#if USE_MEMORY_SHARING

        // added 2024
        //shared memory start
        
        sem.wait();

        std::stringstream ss(shared_memory);
        std::string detection_str = ss.str();
        sem.post();
            }

        std::regex tuple_regx("\\(([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)\\)");

        std::sregex_iterator it(detection_str.begin(), detection_str.end(), tuple_regx);
        std::sregex_iterator end;
        auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        try {
            float bot_num = 0;remap
            float tim_gap = 0;
            for (; it != end; ++it) {
                std::string first_element = (*it)[1].str();
                std::string second_element = (*it)[2].str();
                std::string third_element = (*it)[3].str();
                std::string fourth_element = (*it)[4].str();
                std::string fifth_element = (*it)[5].str();
                std::string sixth_element = (*it)[6].str();
                std::string seventh_element = (*it)[7].str();
                int id = std::stoi(first_element);
                long long time_msec = std::stoll(second_element);
                int x = std::stoi(third_element);
                int y = std::stoi(fourth_element);
                float theta = std::stof(fifth_element);
                float speed = std::stof(sixth_element);
                int camera_id = std::stoi(seventh_element);
                long diffrence = current_time - time_msec;
                if (id != -1) {
                    bot_num = bot_num + 1;
                    tim_gap = tim_gap + diffrence;
                    //std::cout << "recived id: " << id << ", recived_time: " << time_msec << ", recived_x: " << x << ", recived_y: " << y << ", recived_theta: " << theta << ", recived_speed: " << speed << ", recived_camera_id: " << camera_id << ", gap: " << diffrence << std::endl;
                }
            }
            if (bot_num != 0) {
                avg_time_gap = tim_gap / bot_num;
                //std::cout << "Number of bots detected: " << bot_num << std::endl;
                //std::cout << " time_gap: " << tim_gap << std::endl;
            } else {
                avg_time_gap = -1;
            }
            if (std::isnan(avg_time_gap)) {
                avg_time_gap = -1;
            }
            //std::cout << "AVG_Time_gap: " << avg_time_gap << std::endl;
            //std::cout << "Time_gap: " << tim_gap << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << '\n';
            avg_time_gap = -1;
        }
        //std::cout <<avg_time_gap << std::endl;

#endif

        auto detectionTime = std::chrono::system_clock::now().time_since_epoch();
        uint64_t detectionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(detectionTime).count();

        boost::posix_time::ptime import_start = boost::posix_time::microsec_clock::universal_time();

        auto init_start = std::chrono::high_resolution_clock::now();

        // Start measurement
        LogTime consumer_wait_timer;
        
        // Wait for the producer to produce a frame
        while (fast_consumer_counter[camera_id].load() == producer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

#if ENABLE_FAST_LOGS
        consumer_wait_timer.stop_ms("Fast thread waiting for producer", file_output);
#endif

        // Atomically load the published index with memory synchronization
        int index = producer_counter[camera_id].load(std::memory_order_acquire);

        fast_consumer_counter[camera_id] = index;

        FrameData& frame_data = buffer[camera_id][index];
        cv::Mat frame = frame_data.frame;
        LogTime frametime = frame_data.frametime;

        // Check if the frame is empty
        if (frame.empty()) {
            std::cout << "No frame recieved in fast thread, camera " << camera_id << std::endl;
            break;
        }

        LogTime transform_frame_timer;

        bool frame_captured = transform_frame(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            std::cout << "no frame captured (fast), exiting. Camera " << camera_id << std::endl;
            // exit(1);
            return -1;
        }

        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        }; 
        
#if ENABLE_FAST_LOGS
        transform_frame_timer.stop_ms("transform_frame", file_output);
        fast_thread_logger.log_operation(DebugLogger::TRANSFORM_TIME, transform_frame_timer.stop_us(), fast_consumer_counter[camera_id].load());
#endif

        // zarray_t *detections = apriltag_detector_detect(detector, &im);
        zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*)); //2023: from FastSearch-code

        global_search_counter++;

        use_exhaustive_search = false;

        auto init_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(init_end - init_start).count();

        // file_output << "Initialize time: " << std::fixed << std::setprecision(2) << init_duration / 1000.0 << " ms" << endl;

        fast_thread_logger.log_operation(DebugLogger::INITIALIZE_TIME, init_duration, fast_consumer_counter[camera_id].load());

        LogTime fast_Search_timer;
        std::copy(std::begin(tags), std::end(tags), previous_tags);
        boost::posix_time::ptime latest_frame = boost::posix_time::microsec_clock::universal_time();
        fast_search(im, latest_frame, v_max, a_max, alpha,
                        min_search_dim, CAM_NAME, time_uncertainty, detector,
                        detections, tags, use_exhaustive_search, file_output);
#if ENABLE_FAST_LOGS
        fast_Search_timer.stop_ms("Fast search in fast thread", file_output);
        fast_thread_logger.log_operation(DebugLogger::PART_SEARCH_TIME, fast_Search_timer.stop_us(), fast_consumer_counter[camera_id].load());
#endif

        LogTime process_timer;

        // std::cout << !use_exhaustive_search << zarray_size(detections) << 
        // (!use_exhaustive_search && zarray_size(detections) != 0) << std::endl;

        if (!use_exhaustive_search && zarray_size(detections) != 0) {

            // Get time of frame/detection----------------

            Message buf {
                htobe32(1) /* type */ ,
                htobe32(2) /* subtype */,
                htobe32(seq) /* seq */,
                // added 2024
                htobe64(detectionTime_ms), /* detection timestamp */
                htobe64(avg_time_gap) /* avg time gap */
            };
            // Camera coordinates for tag center.
            std::vector<at::Point> camera_detections(zarray_size(detections)); 
            // Camera coordinates for tag corners.
            std::vector<at::Point> camera_corner_detections(2*zarray_size(detections)); 

            // is this one even relevant? elias2025
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;
                camera_detections[i] = at::Point(dd->c[0], dd->c[1]);
                camera_corner_detections[2*i] = at::Point(dd->p[0][0], dd->p[0][1]);
                camera_corner_detections[2*i+1] = at::Point(dd->p[3][0], dd->p[3][1]);
                // Aron: Adjust for part image coordinates?

                // elias2025 >>> ADD UNDISTORTION OF POINTS HERE



                



                camera_detections[i].x += tag->area.x_start;
                camera_detections[i].y += tag->area.y_start;
#if PRINT_DEBUG_MSG           

                    // file_output <<"CAM#"<<CAM_NAME<<": " 
                    //           << "Found when using PART_IMAGE. X: " 
                    //           << camera_detections[i].x << " Y:" 
                    //           << camera_detections[i].y << endl;
                    file_output <<"CAM#"<<CAM_NAME<<" " 
                            << "Found when using PART_IMAGE. X = " 
                            << camera_detections[i].x << ", Y = " 
                            << camera_detections[i].y << std::endl;
#endif         
            }
            // Room coordinates for tag center.
            std::vector<at::Point> room_detections(zarray_size(detections)); 
            // Room coordinates for tag corner.
            std::vector<at::Point> room_corner_detections(2*zarray_size(detections)); 

            static boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));

            buf.cam_id = htobe32(CAM_NAME);
            int n_detections = zarray_size(detections);
            std::vector<cv::Point2f> points;

            float max_temp_alpha = 0.0f;
            float max_temp_a = 0.0f;

            for (int i = 0; i < n_detections; i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;

                Tag* previous_tag = previous_tags + dd->id;

                // elias2025 >>> CONVERT TO GLOBAL COORDINATES HERE

                int x_area_start = tag->area.x_start;
                int y_area_start = tag->area.y_start;
                cv::Mat distorted_points = (cv::Mat_<double>(4,2) << dd->p[3][0] + x_area_start, dd->p[3][1] + y_area_start,
                                                                     dd->p[2][0] + x_area_start, dd->p[2][1] + y_area_start,
                                                                     dd->p[1][0] + x_area_start, dd->p[1][1] + y_area_start,
                                                                     dd->p[0][0] + x_area_start, dd->p[0][1] + y_area_start);
                cv::Mat undistorted_points = undistort_points(distorted_points, camera_id);
#if ELIAS_PRINT
                std::cout << "distorted_points = " << distorted_points << " undistorted_points = " << undistorted_points << std::endl;
#endif
                // GLOBAL COORDINATION CALCULATION

                cv::Mat world_position, world_rotation;
                world_position = estimate_object_global_position(camera_id, undistorted_points, &world_position, &world_rotation, frame);
#if ELIAS_PRINT
                std::cout << "world position = " << world_position << std::endl << std::endl << std::endl;//" world rotation = " << world_rotation << std::endl;
                std::cout << "camera id = " << camera_id << std::endl;
#endif
                tag->world_position = world_position;
                cv::Point2f* cornerDetection = 2*i + room_corner_detections.data();
                cv::Point2f* detection = i + camera_detections.data();
                update_tag(detection, cornerDetection, latest_frame, tag, file_output); // change update tag so that it takes in the world position and rotation as well and stores it in the tag
                detection = i + room_detections.data();
                // ELIAS2025
                float global_x_m = float (world_position.at<double>(0)); // gets meter coordinates of x axis
                float global_y_m = float (world_position.at<double>(1)); // gets meter coordinates of y axis
                float global_z_m = float (world_position.at<double>(2)); // gets meter coordinates of z axis
                // ELIAS2025 implement z axis as well and then rotation
                
                add_detection_to_msg(dd->id, frametime.timestamp(), global_x_m, global_y_m, global_z_m, //tag->x, tag->y
                                    tag->theta, i, CAM_NAME, buf);   // added 2024, "detectionTime_ms" added
                
                max_temp_alpha = std::max(max_temp_alpha, std::abs(tag->theta));

                if (tag->valid_velocity && previous_tag->valid_velocity) {


                    auto elapsed = tag->latest_detection - previous_tag->latest_detection;
                    float us = elapsed.total_microseconds();
                    float time_s = us / 1e6 + time_uncertainty;

                    float temp_a = std::abs((tag->velocity - previous_tag->velocity) / time_s);
                    max_temp_a = std::max(max_temp_a, temp_a);

#if PRINT_DEBUG_MSG           

                    // file_output << "Previous velocity: " << previous_tag->velocity << " Current velocity: " << tag->velocity << " time s: " << time_s << " acceleration: " << temp_a << endl;
                    file_output << "Previous velocity = " << previous_tag->velocity << ", Current velocity = " << tag->velocity << ", time s = " << time_s << ", acceleration = " << temp_a << std::endl;

#endif

                }
                    // max_temp_v = max(max_temp_v, tag->velocity); 

            // First create an apriltag_detection_info_t struct using your known parameters.
                apriltag_detection_info_t info;
                info.det = dd;
                info.tagsize = opts.tag_size;


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
                        // default value or error handling
                        throw std::invalid_argument("Unsupported resolution");
                    }
                } else {
                    throw std::invalid_argument("Unsupported device number");
                }



                
            }

            // added 2025

#if USE_EWMA
            alpha = angle_tracker.get_max_value(5);

            angle_tracker.add_number(max_temp_alpha);

            a_max = acceleration_tracker.get_max_value(5);

            acceleration_tracker.add_number(max_temp_a);
#endif



            for(int i = 0; i<100; i++){
                if(sem_1.try_wait()){
                    break;
                }
                usleep(1);
            }
            buf.length = htobe32(n_detections);
            memcpy(&(ptr->msg), &buf, sizeof(buf));
            ptr->flag = 1;
            sem_1.post();

#if ENABLE_FAST_LOGS
            frametime.stop_ms("Latency fast", file_output);
#endif

        }

        //ELIAS2025 add in so that it shares the position of the vehicle when inside the area between two cameras 
        // and add into the consume_buffers so that it reads the position and does fast search on that area

        // Producer: writes its own detection results to the neighbouring camera's buffer
        for (int i = 0; i < 2 && produce_buffers[i]; ++i) {
            for (int id = 0; id < MAX_TAG_ID; ++id) {
                if (tags[id].is_detected) {  // Tag is detected
                    int y = tags[id].y;
                    float world_x = (float) (tags[id].world_position.at<double>(0)); 
                    float world_y = (float) (tags[id].world_position.at<double>(1)); 
                    // std::cout << "world_x " << tags[id].world_position << " world_y" << world_y << std::endl;

                    // file_output << "Tag#" << id << ": x=" << tags[id].x << ", y=" << tags[id].y << endl;

                    // Check if y is in the overlap area
                    if (y >= overlap_ranges_1080p[camera_id][i].min_y && y <= overlap_ranges_1080p[camera_id][i].max_y) {
                        // std::cout << "has sent message" << std::endl;
                        OverlapTagInfo info{id, tags[id], a_max, alpha, tags[id].latest_detection};

#if PRINT_DEBUG_MSG           
                        file_output << "Tag#" << id << " detected in the overlapping area. Time frame: " << tags[id].latest_detection << std::endl;
#endif
                        produce_buffers[i]->produce(info);
                    }
                }
            }
        }


        // Consumer：read data from adjacent camera's buffer
        for (int i = 0; i < 2 && consume_buffers[i]; ++i) {
            OverlapTagInfo incoming;
            while (consume_buffers[i]->consume(incoming)) {
                boost::posix_time::ptime current_frame = boost::posix_time::microsec_clock::universal_time();
                boost::posix_time::time_duration frame_duration = current_frame - incoming.timestamp; // 两个ptime相减
                long frame_duration_ms = frame_duration.total_milliseconds(); // 转换为毫秒
                
                // detect time difference and Tag status
                if (frame_duration_ms <= 5 && !tags[incoming.tag_id].is_detected) { // 
                    // std::cout << "has gotten message with tag with position " << std::endl; //<< new_tag.world_position
                    use_exhaustive_search = true;
                    a_max = incoming.a_max;
                    alpha = incoming.alpha;
                    // Tag new_tag = incoming.tag;
                    // cv::Mat incoming_distorted_points = global_to_pixel(camera_id, new_tag.world_position, frame);
                    // new_tag.x = (float) incoming_distorted_points.at<double>(0);
                    // new_tag.y = (float) incoming_distorted_points.at<double>(1);
                    // tags[incoming.tag_id] = new_tag;
                    // std::cout << "New tag added for camera " << camera_id << " at world position " << new_tag.world_position << " at pixel values x " << new_tag.x << " y " << new_tag.y << std::endl;
#if PRINT_DEBUG_MSG           
                    file_output << "Missing detection of Tag#" << incoming.tag_id << " in the overlapping area. Frame duration time: " << frame_duration_ms << std::endl;
#endif
                    break;
                }
            }
        }

        //If there are tags missing, use exhaustive search next time
        for(Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++){

            if (!tag->is_detected && tag_exists(tag->x, tag->y)) {
                use_exhaustive_search = true;
#if PRINT_DEBUG_MSG           
                file_output << "Tags are missing." << std::endl;
#endif
            }
            tag->is_detected = false; //clears variable for next search
        }

#if ENABLE_FAST_LOGS           
        process_timer.stop_ms("Process time", file_output);
        fast_thread_logger.log_operation(DebugLogger::PROCESS_TIME, process_timer.stop_us(), fast_consumer_counter[camera_id].load());
#endif

        bool have_exhaustive_searched = false;

        DetectionData detection_data;

        if (use_exhaustive_search || global_search_counter == GLOBAL_SEARCH_MIN) {

            LogTime global_search_timer;

#if PRINT_DEBUG_MSG
            file_output <<"CAM#"<<CAM_NAME<<" " << "Using GLOBAL SEARCH ##############################################\n";
#endif
            loop_count = 1;
            global_search_counter = 0;
            // use_exhaustive_search = false;

            have_exhaustive_searched = true;

            std::copy(std::begin(tags), std::end(tags), previous_tags);

            //Clear previous coordinates of all tags.
            for (Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++) {
                reset_tag(im.width, im.height, tag);
            }

            while(search_consumer_counter[camera_id].load() == search_producer_counter[camera_id].load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            search_consumer_counter[camera_id] = search_producer_counter[camera_id].load();
            detection_data = search_buffer[camera_id][search_consumer_counter[camera_id].load()];

            std::copy(std::begin(detection_data.tag_data), std::end(detection_data.tag_data), tags);

#if ENABLE_FAST_LOGS
            global_search_timer.stop_ms("Global search in fast thread", file_output);
            // fast_thread_logger.log_operation(DebugLogger::GLOBAL_SEARCH_TIME, global_search_timer.stop_us(), fast_consumer_counter[camera_id].load());
#endif

            float max_temp_alpha = 0.0f;
            float max_temp_a = 0.0f;

            bool no_detected = true;

            for (int i = 0; i < MAX_TAG_ID; i++) {
                if (detection_data.tags[i].found) {

                    no_detected = false;

                    max_temp_alpha = std::max(max_temp_alpha, std::abs(tags[i].theta));

                    if (tags[i].valid_velocity && previous_tags[i].valid_velocity) {

                        auto elapsed = tags[i].latest_detection - previous_tags[i].latest_detection;
                        float us = elapsed.total_microseconds();
                        float time_s = us / 1e6 + time_uncertainty;

                        float temp_a = std::abs((tags[i].velocity - previous_tags[i].velocity) / time_s);
                        max_temp_a = std::max(max_temp_a, temp_a);

#if PRINT_DEBUG_MSG
                        file_output << "Previous velocity = " << previous_tags[i].velocity << ", Current velocity = " << tags[i].velocity << ", time s = " << time_s << " acceleration: " << temp_a << std::endl;
#endif
                    }

#if PRINT_DEBUG_MSG
                file_output <<"CAM#"<<CAM_NAME<<" " 
                        << "Found when using GLOBAL_IMAGE. X = " 
                        << detection_data.tags[i].camera_coords->x << " Y = " 
                        << detection_data.tags[i].camera_coords->y << std::endl;
#endif
                }
            }

#if USE_EWMA
            if (!no_detected) {
                    alpha = angle_tracker.get_max_value(5);

                    angle_tracker.add_number(max_temp_alpha);

                    a_max = acceleration_tracker.get_max_value(5);

                    acceleration_tracker.add_number(max_temp_a);
            }

            if (detection_data.buf) {    

                for(int i = 0; i<100; i++){
                    if(sem_1.try_wait()){
                        break;
                    }
                    usleep(1);
                }
                memcpy(&(ptr->msg), &detection_data.buf, sizeof(detection_data.buf));
                ptr->flag = 1;
                sem_1.post();

#if ENABLE_FAST_LOGS
                frametime.stop_ms("Latency exhaustive", file_output);
#endif
                detection_data.clearMessage();
            }
#endif
        }

        if (not opts.no_gui) {
            have_exhaustive_searched? update_exhaustive_gui(detection_data, tags, frame, avg_hz) : update_gui(detections, tags, frame, avg_hz, file_output);
            if (opts.mirror_display) {
                cv::flip(frame, frame, 1);
            }
        
            cv::imshow(win, frame);
            cv::waitKey(1);
        }

        if (sig_stop) {
            break;
        }
        apriltag_detections_destroy(detections);

        loop_count++;

        boost::posix_time::ptime import_end = boost::posix_time::microsec_clock::universal_time();

        uint32_t tot_time = (import_end - import_start).total_milliseconds();
        
        sum_hz += tot_time;

        if (hz_counter == 2*FPS) {
            avg_hz = 1 / (static_cast<float>(tot_hz)/(2*FPS*1000));

#if ENABLE_FAST_LOGS
            file_output << "Frequency: " << avg_hz << " Hz"  << std::endl;
#endif
            hz_counter = 0;
            sum_hz = 0;
        }
        hz_counter++;

        // End measurement
        int elapsed_time = timer.stop_us();
#if ENABLE_FAST_LOGS
        file_output << "Loop: count=" << total_loop_count - 1 << ", trial=" << trial << ", Duration=" << timer.stop_ms() << " ms" << std::endl;;

        // Maybe remove?
        fast_thread_logger.log_operation(DebugLogger::LOOP_TIME, elapsed_time, fast_consumer_counter[camera_id].load());
        fast_thread_logger.write_to_file_if_needed(elapsed_time, "fast-producer", filename.str());
#endif
        }
    }
    apriltag_detector_destroy(detector);
    file_output.close();

    std::cout << "Camera " << camera_id << " fast exiting" << std::endl;

    return 0;
}