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

#include "DebugLogger.hpp"

DebugLogger::DebugLogger(size_t max_entries = 16000) {
    buffer_capacity = max_entries;
    log_buffer = new LogEntry[buffer_capacity];
    write_index = 0;
    is_buffer_full = false;
}

void DebugLogger::log_operation(Reason reason, uint32_t value, uint8_t counter) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);

    // fill in the log entry
    log_buffer[write_index] = {reason, value, counter, static_cast<uint32_t>(now_time_t)};

    // update the write index, use it as circularly
    write_index = (write_index + 1) % buffer_capacity;
    if (write_index == 0) {
        is_buffer_full = true;
    }
}

void DebugLogger::write_to_file_if_needed(uint32_t loop_time, const std::string& thread_name, const std::string& file_name) {
    if (loop_time > 100000) {
        std::ofstream file(file_name, std::ios::app);
        if (file.is_open()) {
            file << "Thread: " << thread_name << ", Loop time exceeded: " << std::fixed << std::setprecision(2)<< loop_time / 1000.0 << " ms\n";
            // size_t start_index = is_buffer_full ? write_index : 0;
            size_t end_index = is_buffer_full ? buffer_capacity : write_index;

            size_t start_index = thread_name == "fast-producer"? end_index - 5 : end_index - 5;

            // size_t latest_index = is_buffer_full ? buffer_capacity - 1 : write_index - 1;
            // const auto& entry = log_buffer[latest_index];

            for (size_t i = start_index; i < end_index; ++i) {
                const auto& entry = log_buffer[i];
                file << "Reason: " << reason_to_string(static_cast<Reason>(entry.reason))
                    << ", Value: " << std::fixed << std::setprecision(2) << static_cast<int>(entry.value) / 1000.0
                    << ", Counter: " << static_cast<int>(entry.counter)
                    << ", Timestamp: " << entry.timestamp << "\n";
            }
            file << "----------------------------------------\n";
            file.close();
        }
    }
}

std::string DebugLogger::reason_to_string(Reason reason) {
    switch (reason) {
        case TRANSFORM_TIME: return "transform time";
        case CONSUMER_TIME: return "consumer time";
        case INITIALIZE_TIME: return "initialize time";
        case PART_SEARCH_TIME: return "part search time";
        case PROCESS_TIME: return "process time";
        case GLOBAL_SEARCH_TIME: return "global search time";
        case LOOP_TIME: return "loop time";
        default: return "unknown";
    }
}