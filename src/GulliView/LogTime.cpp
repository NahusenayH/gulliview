#include "LogTime.hpp"

// constructor autmatically starts clock
LogTime::LogTime(const std::string& input_name, std::ofstream& input_file) 
                    : name(input_name), file(input_file) {

    start_time = std::chrono::high_resolution_clock::now();
}

// Stop clock and print to log file only if logs are enabled
void LogTime::stop_ms(){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    file << name << ": " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
}
void LogTime::stop_us(){
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    file << name << ": " << std::fixed << std::setprecision(2) << duration << " us" << std::endl;
}