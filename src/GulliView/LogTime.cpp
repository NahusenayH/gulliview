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

#include "LogTime.hpp"

// constructor autmatically starts clock
LogTime::LogTime(const std::string& input_name) : name(input_name) {
    start_time = std::chrono::high_resolution_clock::now();
}

// Stop clock and print to log file only if logs are enabled
void LogTime::stop_ms(std::ofstream& file){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    file << name << ": " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
}
void LogTime::stop_us(std::ofstream& file){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    file << name << ": " << std::fixed << std::setprecision(2) << duration << " us" << std::endl;
}
void LogTime::stop_ns(std::ofstream& file){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    file << name << ": " << std::fixed << std::setprecision(2) << duration << " ns" << std::endl;
}
int LogTime::stop_ms(){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    return int(duration);
}
int LogTime::stop_us(){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    return int(duration);
}
int LogTime::stop_ns(){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    return int(duration);
}