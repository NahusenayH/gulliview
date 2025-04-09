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

#include "ProducerThread.hpp"

void produce_frame(int camera_id, cv::VideoCapture *cap) {

    #if BINDING_CPU_CORES
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
    
        // bind the thread to the corresponding core
        CPU_SET(camera_id + 8, &cpuset);
    
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
    
        std::ostringstream filename;
        filename << "output-producer/camera_" << camera_id << "_output-producer.log";
        std::ofstream file_output(filename.str(), std::ios::out);
    
        while (true)
        {
    
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
                    file_output << "Core number: " << i << std::endl;
                    break;
                }
            }
    #endif
    
            // Start
            auto start = std::chrono::high_resolution_clock::now();
    
            unsigned int next = (producer_counter[camera_id].load() + 1) % BUFFER_SIZE;
    
            while(next == fast_consumer_counter[camera_id].load() && next == nice_consumer_counter[camera_id].load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
    
            *cap >> buffer[camera_id][next];
            producer_counter[camera_id] = next;
    
            // End measurement
            auto end = std::chrono::high_resolution_clock::now();
    
            // Calculation time (in microseconds)
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    
    #if PRINT_DEBUG_MSG
            // printing time
            file_output << "Execution time for the producer: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
    #endif
            // added 2025
            // Increment the shared frame counter
            // shared_frame_count++;
        }
    
        file_output.close();
    
}
    