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

#include "ProcessCamera.hpp"

// Camera processing thread
int process_camera(int camera_id, GulliViewOptions opts) {

    DebugLogger fast_thread_logger(16000); // 16-bit × 16000 = 32KB
    DebugLogger nice_thread_logger(16000);


    // Initialize each to 0, added 2025
    for (int i = 0; i < 4; ++i) {
        producer_counter[i] = 0;
        search_producer_counter[i] = 0;
        search_consumer_counter[i] = 0;
        fast_consumer_counter[i] = 0;
        nice_consumer_counter[i] = 0;
    }

#if PRODUCE_FRAME_MODE == 2

    LockFreeBuffer* lock_buffer = new LockFreeBuffer();  // Initializing pointers
    LockFreeSearchBuffer* lock_search_buffer = new LockFreeSearchBuffer();

#endif

    // Initialize video capture for this camera
    cv::VideoCapture video_capture;
    cv::Mat frame, gray;

#if LIVE_FEED
    // Live feed
    init_video_capture(camera_id, opts.frame_width, opts.frame_height, video_capture, frame);
#else
    // Saved video
    init_video_open(camera_id, opts.frame_width, opts.frame_height, video_capture, frame);
#endif

    std::cout << "Set camera to resolution: "
            << video_capture.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
            << video_capture.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";

    // Aron: purpose of these three?
    at::Point SOURCE_POINTS_PTS[4];
    at::Point DESTINATION_POINTS_PTS[4];
    int CAM_NAME;

    cv::Mat map1, map2;

    automated_calibration(opts.frame_width,
                        opts.frame_height,
                        DESTINATION_POINTS_PTS,
                        SOURCE_POINTS_PTS,
                        &CAM_NAME, 
                        video_capture,
                        map1,
                        map2,
                        camera_id); // Aron: add tag height?

    std::cout << "Camera Name " << CAM_NAME << ": camera id " << camera_id << std::endl; 


    at::Mat pts = cv::getPerspectiveTransform(SOURCE_POINTS_PTS, DESTINATION_POINTS_PTS);
    
    std::string win = "Camera " + std::to_string(CAM_NAME);
    if (!opts.no_gui) {
        cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    }



#if USE_MEMORY_SHARING

    // shared memory-file between membership_service and GulliView start 
    boost::interprocess::named_semaphore sem(boost::interprocess::open_only, "/my_semaphore");

    int fd =shm_open("/my_shared_memory", O_RDWR, 0666);


    if (fd == -1) {
        std::cerr << "shm_open failed" << std::endl;
        return 1;
    }
    struct stat sb;
    if (fstat(fd, &sb) == -1) {buffer
        std::cerr << "faied to get size" << std::endl;
        return 1;
    }

    char* shared_memory = static_cast<char*>(mmap(NULL, sb.st_size, PROT_READ, MAP_SHARED, fd, 0));
    if (shared_memory == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }
    // shared memory-file end


#else

    static boost::interprocess::named_semaphore sem(
        boost::interprocess::open_or_create, "dummy1", 1
    );
    // static boost::interprocess::named_semaphore sem_1(
    //     boost::interprocess::open_or_create, "dummy2", 1
    // );

    char* shared_memory = nullptr;
    // SharedData* ptr = nullptr;

    //********************************************** Shared memory for sending msg to Sender (added 2024) **********************************************
    
    boost::interprocess::named_semaphore sem_1(boost::interprocess::open_or_create, opts.shared_semaphore.c_str(), 1);
    
    //std::cout << "semaphore in Gulliview  " << sem_1 << std::endl;
    std::cout << opts.shared_semaphore.c_str() << std::endl;
    const char *memName = opts.shared_memory.c_str();
    std::cout << opts.shared_memory.c_str() << std::endl;
    const size_t SIZE = sizeof(SharedData);
    int shm_fd = shm_open(memName, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "shm_open failed" << std::endl;
        return 1;
    }

    if(ftruncate(shm_fd, SIZE) != 0) {
        std::cerr << "size set fail" << std::endl;
        return 1;
    }

    SharedData *ptr = (SharedData *)mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }

#endif


    //********************************************** Shared memory for sending msg to Sender (added 2024) **********************************************


# if ! RUN_ONLY_PRODUCER
    std::thread nice_consumer(
        nice_consume_frame, 
        CAM_NAME, 
        std::ref(sem), 
        std::ref(sem_1), 
        shared_memory, // Passing raw pointers is fine
        ptr, // Assuming ptr is already a pointer
        frame,
        gray,
        camera_id, 
        map1, 
        map2, 

        // tags,
        opts, // Pass by reference if it's non-copyable or costly to copy
        win, // `std::string` should be wrapped in std::ref if mutable
        std::ref(nice_thread_logger)
    );

    std::thread fast_consumer(
        fast_consume_frame, 
        CAM_NAME,
        CAM_NAME,//thread_id,
        std::ref(sem), 
        std::ref(sem_1), 
        shared_memory, // Passing raw pointers is fine
        ptr, // Assuming ptr is already a pointer
        frame,
        gray,
        camera_id, 
        map1, 
        map2, 

        // tags,
        opts, // Pass by reference if it's non-copyable or costly to copy
        win,
        std::ref(fast_thread_logger)
    );
# endif

    // std::thread producer(produce_frame, camera_id, &video_capture);
    //edited to handle empty frames
    std::thread producer(produce_frame, camera_id, &video_capture, opts.frame_width, opts.frame_height);
# if ! RUN_ONLY_PRODUCER
    nice_consumer.join();
    // fast_consumer.join();
# endif
    
    producer.join();
    std::cout << "All threads joined camera " << camera_id << std::endl;
    return 0;
}
