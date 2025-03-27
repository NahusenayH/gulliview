#ifndef _ANGLETRACKER_H_
#define _ANGLETRACKER_H_

#include <cmath>
#include <chrono>
#include <deque>

class AngleTracker {
private:
    struct DataPoint {
        float value; // The value of the data
        std::chrono::steady_clock::time_point timestamp; // The time the data was added
    };

    std::deque<DataPoint> last_10_numbers; // Stores the last 10 valid numbers with timestamps
    float average; // Running average of valid numbers

public:
    AngleTracker();
    void add_number(float number);
    float get_average() {return average;}
    float get_max_value(int epsilon);

private:
    void remove_old_elements();
    void update_average(float number);
};

#endif