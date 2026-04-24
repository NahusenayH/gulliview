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

#ifndef _LOGTIME_H_
#define _LOGTIME_H_

/*
Created to reduce clutter from reused lines and to increase readability
*/

#include <chrono>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <ostream>

class LogTime {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::string name;
    uint64_t time;

public:
    LogTime();

    uint64_t timestamp(){
        return time;
    }

    // Stop clock and print to log file
    void stop_ns(const std::string& name, std::ofstream& file){
        file << name << ": " << stop_ns() << " ns" << std::endl;
    };
    void stop_us(const std::string& name, std::ofstream& file){
        file << name << ": " << stop_us() << " us" << std::endl;
    };
    void stop_ms(const std::string& name, std::ofstream& file){
        file << name << ": " << stop_ms() << " ms" << std::endl;
    };
    void stop_s(const std::string& name, std::ofstream& file){
        file << name << ": " << stop_s() << " s" << std::endl;
    };

    // Stop the clock
    double stop_ns();
    double stop_us() {
        return std::round(stop_ns())/1000.0;
    };
    double stop_ms() {
        return std::round(stop_us())/1000.0;
    };
    double stop_s() {
        return std::round(stop_ms())/1000.0;
    };   
};

#endif