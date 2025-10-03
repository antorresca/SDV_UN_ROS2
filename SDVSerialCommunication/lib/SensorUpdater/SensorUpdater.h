#ifndef __SENSOR_UPDATER_H__
#define __SENSOR_UPDATER_H__

#include <Arduino.h>

class SensorUpdater
{
public:
    SensorUpdater(void (*)(void), void (*)(void), unsigned long);
    void updateTasK();
    void updateData();
    void printData();
    void attachPrintTask(void (*)(void));
    void attachUpdateSensorTask(void (*)(void));
    void enableSensorFeedback(bool);
    bool isSensorFeedbackEnabled();

    static void updateTime();
    static void enableAllMessages(bool);

private:
    void (*update_data_task)(void);
    void (*print_data_task)(void);

    unsigned long last_sensor_time_stamp = 0; // Time stamp of last sended
    unsigned long sensor_print_rate;         // Update rate, in milli seconds
    bool sensor_data_updated = false;         // Flag that indicates if IMU data is updated
    bool enable_sensor_feedback = false;      // Flag to control sending of IMU message

    static bool enable_all_messages;
    static unsigned long task_time;
};
#endif // __SENSOR_UPDATER_H__