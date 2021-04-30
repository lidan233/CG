//
// Created by lidan on 2021/4/30.
//

#ifndef CG_WATCH_H
#define CG_WATCH_H

#include <time.h>

class Watch {
public:
    Watch();
    void start();
    void stop();
    void reset();
    double getTime();    // in seconds

private:
    bool isStarted = false;
    timespec ts_begin, ts_end;
    double timeRunning = 0.0;
};


#endif //CG_WATCH_H
