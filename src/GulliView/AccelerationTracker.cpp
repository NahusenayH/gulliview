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

#include "AccelerationTracker.hpp"

// Constructor initializes average to 0
AccelerationTracker::AccelerationTracker() : average(0.0) {}
    
// Method to add a number to the tracker (only if the number is greater than the current average)
void AccelerationTracker::add_number(float number) {
    // Use absolute value for the number
    float abs_number = std::abs(number);

    // Compare the absolute value of the new number with the absolute value of the average
    if (abs_number > std::abs(average)) {
        // Add the new data point with the current time
        last_10_numbers.push_back({abs_number, std::chrono::steady_clock::now()});

        // Remove elements older than 5 seconds
        remove_old_elements();

        // If the deque exceeds 10 elements, remove the oldest
        if (last_10_numbers.size() > 10) {
            last_10_numbers.pop_front();
        }

        // Update the average using the new formula
        update_average(abs_number);
    }
}

// Method to get the maximum value among the last 10 numbers
float AccelerationTracker::get_max_value(int epsilon) {
    // Find the maximum value in the deque
    float max_in_deque = 0;
    for (const auto& data : last_10_numbers) {
        max_in_deque = std::max(max_in_deque, data.value);
    }

    // Calculate the adjustment
    float adjustment = float(epsilon) / 100 * (max_in_deque + 1);

    return max_in_deque + adjustment;
}


// Method to remove elements older than 5 seconds
void AccelerationTracker::remove_old_elements() {
    auto now = std::chrono::steady_clock::now();
    while (!last_10_numbers.empty() &&
            std::chrono::duration_cast<std::chrono::seconds>(now - last_10_numbers.front().timestamp).count() > 5) {
        last_10_numbers.pop_front();
    }
}

// Method to update the average
void AccelerationTracker::update_average(float number) {
    float simple_average = (std::abs(average) + std::abs(number)) / 2.0;
    average = simple_average;
}