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
#include <opencv2/imgproc.hpp>
#include "ProducerThread.hpp"

#include <csignal>
static volatile bool pause_clone = false;

void toggle_clone(int) {
    pause_clone = !pause_clone;
    std::cout << "[BENCHMARK] Clone " << (pause_clone ? "PAUSED" : "RESUMED") << std::endl;
}

// void produce_frame(int camera_id, cv::VideoCapture *cap) {
void produce_frame(int camera_id, cv::VideoCapture *cap, int frame_width, int frame_height) {
    #if BINDING_CPU_CORES
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
    
        // bind the thread to the corresponding core
        int thread_num = PRODUCER_THREAD_NUM + camera_id % PRODUCER_THREAD_COUNT * PRODUCER_THREAD_COUNT / 4;
        CPU_SET(thread_num, &cpuset);
    
        // set the CPU affinity of the thread
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
            return;
        }
    #endif
    
        struct sched_param param;
        int policy;
    
        // Set the priority to the highest (the maximum priority of SCHED_RR)
        param.sched_priority = sched_get_priority_max(SCHED_RR);
    
        // Set the scheduling policy to SCHED_RR
        if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
            std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
            perror("Error details");
            return;
        }
    
        // Get the scheduling policy and priority of the current thread (confirm whether the setting is successful)
        if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            return;
        }
        
        //recovery variables for handling empty frames
        constexpr auto kMaxEmptyGap = std::chrono::milliseconds(100);
        constexpr int kMaxReopenAttempts = 5;
        int reopen_failures = 0;
        auto last_success_time = std::chrono::steady_clock::now();
        
        std::ostringstream filename;
        filename << "output/camera_" << camera_id << "_output-producer.log";
        std::ofstream file_output(filename.str(), std::ios::out);
    
        // Register SIGUSR1 to toggle cloning
        signal(SIGUSR1, toggle_clone);
        
        while (true)
        {
            LogTime producer_timer;
    
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
            #if ENABLE_PRODUCER_LOGS
                                file_output << "Core number: " << i << std::endl;
            #endif
                                break;
                            }
                        }
                #endif
                
            unsigned int next = (producer_counter[camera_id].load() + 1) % BUFFER_SIZE;
    
            while(next == fast_consumer_counter[camera_id].load() && next == nice_consumer_counter[camera_id].load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            
            // Init frame
            cv::Mat raw_frame;
                
            // Time for frame capture
            LogTime get_frame_timer;
            
            // Capture the frame from the camera and get timestamp
            *cap >> raw_frame;





            // BENCHMARK CONTROLS - change these to switch test cases
            #define BENCHMARK_RESIZE    true   // true = 1080p, false = 4K
            #define BENCHMARK_30FPS     true   // true = 30fps, false = 60fps

            #if BENCHMARK_RESIZE
            if (!raw_frame.empty() && raw_frame.cols == 3840 && raw_frame.rows == 2160) {
                cv::resize(raw_frame, raw_frame, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
            }
            #endif

            #if BENCHMARK_30FPS
            static int skip_counter = 0;
            if (++skip_counter % 2 == 0) {
                continue;
            }
            #endif






            // // Temporary test: force empty frame every 200 frames
            // static int frame_counter = 0;
            // if (++frame_counter % 200 == 0) {
            //     std::cout << "frame_counter " << frame_counter << " stream reopened" << std::endl;
            //     raw_frame = cv::Mat(); // force empty
            // }
            


            // Downscale 4K to 1080p to reduce memory usage
            // if (!raw_frame.empty() && raw_frame.cols == 3840 && raw_frame.rows == 2160) {
            //     cv::resize(raw_frame, raw_frame, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
            // }

            // Temporary: confirm resize is working
            static bool printed = false;
            if (!printed && !raw_frame.empty()) {
                std::cout << "Frame size after resize: " << raw_frame.cols << "x" << raw_frame.rows << std::endl;
                printed = true;
            }

            //handle empty frames
            if (raw_frame.empty()) {
            # if LOOP_RECORDING
                cap->set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            # else
                auto now = std::chrono::steady_clock::now();
                if (now - last_success_time <= kMaxEmptyGap) {
                    std::cerr << "Camera " << camera_id << " empty frame detected, skipping" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }

    
            ++reopen_failures;
            std::cerr << "Camera "  << camera_id << " no frames for "
                                    << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_success_time).count()
                                    << " ms, reopening (attempt "
                                    << reopen_failures << " of " << kMaxReopenAttempts << ")" << std::endl;

            cap->release();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            if (!open_camera_device(camera_id, frame_width, frame_height, *cap)) {
                if (reopen_failures >= kMaxReopenAttempts) {
                    std::cerr << "Camera " << camera_id << " failed to recover after "
                              << kMaxReopenAttempts << " reopen attempts" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            std::cout << "Camera " << camera_id << " stream reopened" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
# endif
}
reopen_failures = 0;
last_success_time = std::chrono::steady_clock::now();


LogTime frametime;
# if LOOP_RECORDING
            if (raw_frame.empty()) {
                cap->set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
# endif


// Print RSS every 30 seconds
static auto last_print = std::chrono::steady_clock::now();
auto now_time = std::chrono::steady_clock::now();
if (camera_id == 0 && std::chrono::duration_cast<std::chrono::seconds>(now_time - last_print).count() >= 30) {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.find("VmRSS") != std::string::npos || 
            line.find("VmSize") != std::string::npos) {
            std::cout << "[BENCHMARK] " << line << std::endl;
        }
    }
    last_print = now_time;
}



# if RUN_ONLY_PRODUCER
            continue;
# endif








#if ENABLE_PRODUCER_LOGS
            get_frame_timer.stop_ms("Get frame", file_output);
#endif

            // Create struct saving timestamp when frame was capured, used for latency evaluation
            FrameData frame_data;
            frame_data.frametime = frametime;


            // frame_data.frame = raw_frame.clone();
            if (!pause_clone) {
                frame_data.frame = raw_frame.clone();
            } else {
                frame_data.frame = cv::Mat();  // empty mat, no allocation
            }

            // frame_data.frame = cv::Mat();

            // Store the frame data in the buffer
            buffer[camera_id][next] = frame_data;

            // Atomically publish the index, ensuring memory is fully visible
            producer_counter[camera_id].store(next, std::memory_order_release);
    
#if ENABLE_PRODUCER_LOGS
            producer_timer.stop_ms("Produce frame", file_output);
#endif
            // Check if the frame is empty
            if (raw_frame.empty()) {
                std::cout << "No frame captured on camera " << camera_id << std::endl;
                break;
            }
        }
    
        file_output.close();
        std::cout << "Camera " << camera_id << " producer exiting" << std::endl;
}
    