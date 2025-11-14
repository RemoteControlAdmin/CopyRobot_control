# include "utils/delaytime_cal.hpp"


double cal_delay_time(int64_t send_time){
    std::chrono::high_resolution_clock::time_point current_clock = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds nano_current_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch());
    double delay_time = (nano_current_clock.count() - send_time)/ 1000000.0;
    return delay_time;
}