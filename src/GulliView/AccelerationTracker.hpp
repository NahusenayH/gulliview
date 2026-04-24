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

#ifndef _ACCELERATIONTRACKER_H_
#define _ACCELERATIONTRACKER_H_

#include <cmath>
#include <chrono>
#include <deque>

class AccelerationTracker {
private:
    struct DataPoint {
        float value; // The value of the data
        std::chrono::steady_clock::time_point timestamp; // The time the data was added
    };

    std::deque<DataPoint> last_10_numbers; // Stores the last 10 valid numbers with timestamps

    float average; // Running average of valid numbers  

public:
    AccelerationTracker();
    void add_number(float number);
    float get_average() { return average; }
    float get_max_value(int epsilon);

private:
    void remove_old_elements();
    void update_average(float number);

};

#endif


