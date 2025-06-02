/*********************************************************************
 * This file is written as part of the membership development project 2024.
 * An IPC shared memory is used to unifiy the messages and one thread broadcast 
 * detection messages to robots instead of each instance of GulliView.
 *
 * Original authors:
 * Copyright (c) Ali Shirzad  <alishir@chalmers.se>
 * Copyright (c) Ali Solstani <aliso@chalmers.se>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense
 * and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 ********************************************************************/

#include "array.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <endian.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/asio.hpp>
#include <array>
#include <iomanip>
#include <fstream>
#include <string>

// Define your structures
typedef struct __attribute__ ((packed)) DetectionMessage {
    uint32_t id;
    uint64_t time_msec;    
    uint32_t x;
    uint32_t y;
    uint32_t z;
    float theta;
    float speed;
    uint32_t camera_id; // Thread ID added here
} DetectionMessage;

typedef struct __attribute__ ((packed)) Message {
    uint32_t type;
    uint32_t subtype;
    uint32_t seq;
    uint64_t time_msec;
    uint64_t avg_time_gap;   
    uint32_t cam_id;    
    uint32_t length;
    DetectionMessage detections[11];
} Message;

struct SharedData {
    int flag; // 0 means empty, 1 means full
    Message msg; // String data
};

void updateMessage(Message& sharedMsg, Message& updatedMsg) {
    for (int i = 0; i < 11; i++) {
        int tag_id = htobe32(sharedMsg.detections[i].id);
        uint64_t recived_time = htobe64(sharedMsg.detections[i].time_msec);
        uint64_t updated_time = htobe64(updatedMsg.detections[tag_id-4].time_msec);
        if (recived_time > updated_time) {
            updatedMsg.detections[tag_id-4] = sharedMsg.detections[i];
        }
        
    }
}

const char *semNames[] = {"my_semaphore1", "my_semaphore2", "my_semaphore3", "my_semaphore4"};

// Declare semaphores and shared memory pointers as global variables
boost::interprocess::named_semaphore *sems[4];
SharedData *ptrs[4];

// This function will be called when a SIGINT signal is received
void handle_sigint(int sig) {
    for (int i = 0; i < 4; ++i) {
        munmap(ptrs[i], sizeof(SharedData)); 
        sem_unlink(semNames[i]); // Unlink the semaphore
        delete sems[i]; 
        std::cout << "Reader " << i << " cleaned up\n";
    }
    exit(0);
}

// Function to write GTR to csv file for testing 
void writeToFile(double recived_avg_time) {
    std::ofstream file;
    file.open("speed1-2.csv", std::ios_base::app); // Open in append mode
    file << recived_avg_time << "\n"; // Write value and comma, then move to next line
    file.close();
}

using boost::asio::ip::udp;


std::string COL_NAME = "k = 1";
std::ofstream file;
int main() {
    // Set up the signal handler
    signal(SIGINT, handle_sigint);
    const char *memNames[] = {"shared_memory1", "shared_memory2", "shared_memory3", "shared_memory4"};
  

    

    // Open semaphores and shared memory segments
    for (int i = 0; i < 4; ++i) {
        try {
            sems[i] = new boost::interprocess::named_semaphore(boost::interprocess::open_only, semNames[i]);
        } catch (const boost::interprocess::interprocess_exception &ex) {
            std::cerr << "Stupid Error opening semaphore: " << ex.what() << std::endl;
            return -1;
        }

        const size_t SIZE = sizeof(SharedData);
        int shm_fd = shm_open(memNames[i], O_RDWR, 0666);
        if (shm_fd == -1) {
            std::cerr << "Shared memory opening failed\n";
            delete sems[i]; 
            return -1;
        }

        ptrs[i] = (SharedData *)mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (ptrs[i] == MAP_FAILED) {
            std::cerr << "Shared memory map failed\n";
            delete sems[i];
            close(shm_fd);
            return -1;
        }
    }

    time_t startTime = time(nullptr);
    boost::asio::io_service io_service;
    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), "192.168.50.255", "2020");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());
    socket.non_blocking(true);
    socket.set_option(boost::asio::socket_base::broadcast(true));

    Message updated_message;
    //updated_message.avg_time_gap = htobe64(0);
    uint32_t id = 0;

    
    for(int i = 0; i<4; i++){
        updated_message.detections[i].id = 0;
        updated_message.detections[i].time_msec = htobe64(0);
        updated_message.detections[i].x = htobe32(-1);
        updated_message.detections[i].y = htobe32(-1);
        updated_message.detections[i].z = htobe32(-1);
        updated_message.detections[i].theta = -1;
        updated_message.detections[i].speed = -1;
        updated_message.detections[i].camera_id = htobe32(-1);
    }
    int seq = 0;
    float avg_time_gap = 0;
    bool new_message = false;
    while (true) {
        try{
            for (int i = 0; i < 4; i++) {
                int counter = 0;
                if(ptrs[i]->flag == 1){
                    new_message = true;
                    sems[i]->wait();
                    Message msg = ptrs[i]->msg;
                    //std::cout << msg.detections[0].x << std::endl;
                    ptrs[i]->flag = 0;
                    sems[i]->post();
                    updateMessage(msg, updated_message);
                    //std::cout << "counter: " << counter << std::endl;
                    uint64_t updated_avg_time = htobe64(updated_message.avg_time_gap);
                    uint64_t recived_avg_time = htobe64(msg.avg_time_gap);
                    // Write the speed and recived_avg_time_gap to the CSV file
                    
                    /*if (recived_avg_time < 1000){
                        
                        writeToFile(recived_avg_time);
                       
                    }*/
                    //std::cout << "updated_avg_time: " << updated_avg_time << std::endl;
                    //std::cout << recived_avg_time << std::endl;
                    if(recived_avg_time > avg_time_gap){
                        avg_time_gap = recived_avg_time;
                        //std::cout << "new_avg_time: " << avg_time_gap << std::endl;
                    }
              
                }
                else{
                    continue;
                }
            }
            
            if(new_message){
                    seq++;
                    auto send_time = std::chrono::system_clock::now().time_since_epoch();
                    uint64_t send_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(send_time).count();
                    updated_message.type = htobe32(1);
                    updated_message.subtype = htobe32(2);
                    updated_message.seq = htobe32(seq);
                    updated_message.time_msec =  htobe64(send_time_ms);
                    updated_message.length = htobe32(3);
                    updated_message.avg_time_gap = htobe64(avg_time_gap);
                    //std::cout << "Sendt_avg_time: " << htobe64(updated_message.avg_time_gap) << std::endl;
                    //std::cout << "firrst id: " << htobe32(updated_message.detections[1].id) << std::endl;

                    //std::cout << "reader_avg: " << htobe64(updated_message.avg_time_gap) << std::endl;
                    socket.send_to(boost::asio::buffer((uint8_t*) &(updated_message), 256), receiver_endpoint);
                    //std::cout << "seq " << htobe32(updated_message.seq)<< std::endl;
                    new_message =false;
                    avg_time_gap = 0;
                    updated_message.avg_time_gap = htobe64(avg_time_gap);
            }
        } catch (const std::exception& e) {
            std::cerr << "An error occurred: " << e.what() << std::endl;
        }
        //usleep(20000);
    }

    // ----- CLEANUP -----
    for (int i = 0; i < 4; ++i) {
        munmap(ptrs[i], sizeof(SharedData));
        sem_unlink(semNames[i]);
        delete sems[i];
    }

    return 0;
}
