#include <Arduino.h>
#include <TimeWatcherTasks.h>

TimeWatcherTasks::TimeWatcherTasks(
    void (*update_task)(void), 
    void (*stop_task)(void), 
    unsigned long timeout) 
{
    attahcUpdateTask(update_task);
    attahcStopTask(stop_task);
    TimeWatcherTasks::timeout = timeout;
}

void TimeWatcherTasks::attahcUpdateTask(void (*updateTask)(void)) 
{
    TimeWatcherTasks::updateTask = updateTask;
}

void TimeWatcherTasks::attahcStopTask(void (*stopTask)(void)) 
{
    TimeWatcherTasks::stopTask = stopTask;
}

void TimeWatcherTasks::feed(unsigned long time) 
{
    last_update_stamp = time;
    if(watcher_enable) 
        stop_task_executed = false;
}

void TimeWatcherTasks::check(unsigned long time)
{
    if (watcher_enable and !stop_task_executed)
    {
        if (time - last_update_stamp > timeout)
        {
            stopTask();
            stop_task_executed = true;
            //Serial.print("Stop task executed\n\r");
        }
    }
}

void TimeWatcherTasks::enableWatcher(bool enable) 
{
    TimeWatcherTasks::watcher_enable = enable;
    stop_task_executed = false;
}
