#ifndef _DEBUGLOGGER_H_
#define _DEBUGLOGGER_H_


#include <fstream>
#include <iomanip>
#include <chrono>

class DebugLogger {
private:
    struct LogEntry {
        uint8_t reason; // reason enumeration (1 byte)
        uint32_t value; // value (4 bytes)
        uint8_t counter; // current count (max 128, 1 byte)
        uint32_t timestamp; // timestamp (4 bytes)
    };

    LogEntry* log_buffer; // fixed size memory array
    size_t buffer_capacity; // array capacity (16000 entries = 32 KB)
    size_t write_index; // current write index
    bool is_buffer_full; // Flag if the oldest data has been overwritten.

public:
    enum Reason : uint8_t {
        TRANSFORM_TIME = 0,
        CONSUMER_TIME,
        INITIALIZE_TIME,
        PART_SEARCH_TIME,
        PROCESS_TIME,
        GLOBAL_SEARCH_TIME,
        LOOP_TIME
    };

    DebugLogger(size_t max_entries);
    ~DebugLogger() {delete[] log_buffer;}
    void log_operation(Reason reason, uint32_t value, uint8_t counter);
    void write_to_file_if_needed(uint32_t loop_time, const std::string& thread_name, const std::string& file_name);

private:
    std::string reason_to_string(Reason reason);

};

#endif