#include <time.h>
#include <stdint.h>

class RealTimeClock {
    private:
    struct timespec next_time_;

    public:

    RealTimeClock(){
        clock_gettime(CLOCK_MONOTONIC, &next_time_);
    }
    
    void wait_next(long nano_sleeptime) {
        next_time_.tv_nsec += nano_sleeptime;
        if (next_time_.tv_nsec >= 1000000000) {
            next_time_.tv_sec += 1;
            next_time_.tv_nsec -= 1000000000;
        }
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time_, nullptr) != 0) ;
    }

    void reset() {
        clock_gettime(CLOCK_MONOTONIC, &next_time_);
    }

};