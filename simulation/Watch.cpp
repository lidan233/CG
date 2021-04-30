//
// Created by lidan on 2021/4/30.
//

#include "Watch.h"

Watch::StopWatch()
{
}

void Watch::start() {
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts_begin);
    isStarted = true;
}


void Watch::stop() {
    if (!isStarted) {
        return;
    }

    // log current running time
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts_end);
    double time = (ts_end.tv_sec - ts_begin.tv_sec) +
                  (ts_end.tv_nsec - ts_begin.tv_nsec) / 1e9;
    timeRunning += time;
}

void Watch::reset() {
    isStarted = false;
    timeRunning = 0.0;
}

double Watch::getTime() {
    return timeRunning;
}
