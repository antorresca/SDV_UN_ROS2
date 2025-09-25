#include <Arduino.h>
#pragma once

class TimeWatcherTasks
{
public:
    TimeWatcherTasks(void (*)(void), void (*)(void), unsigned long);
    void attahcUpdateTask(void (*)(void));
    void attahcStopTask(void (*)(void));
    void check(unsigned long time);
    void feed(unsigned long time);
    void enableWatcher(bool enable);

private:
    void (*updateTask)(void);
    void (*stopTask)(void);

    unsigned long last_update_stamp;
    bool watcher_enable = false;
    bool stop_task_executed = false;
    unsigned long timeout = 0;

};