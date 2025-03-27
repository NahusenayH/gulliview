#ifndef _LOGTIME_H_
#define _LOGTIME_H_


class LogTime {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::string name;
    std::ofstream& file;
public:
    LogTime(const std::string& input_name, std::ofstream& input_file);
    void stop_ms();
    void stop_us();
};

#endif