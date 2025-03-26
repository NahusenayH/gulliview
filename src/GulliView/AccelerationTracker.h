#ifndef _ACCELERATIONTRACKER_H_
#define _ACCELERATIONTRACKER_H_

#include <cmath>       // For std::abs
#include <chrono>      // For std::chrono::steady_clock
#include <deque>       // For std::deque
#include <algorithm>   // For std::max
#include <iostream>    // For debugging or output (if needed)

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


