#include <Arduino.h>
#include "SensorUpdater.h"

unsigned long SensorUpdater::task_time = 0;
bool SensorUpdater::enable_all_messages = true;

SensorUpdater::SensorUpdater(
    void (*updateSensorTask)(void),
    void (*printTask)(void),
    unsigned long print_rate)
{
    attachUpdateSensorTask(updateSensorTask);
    attachPrintTask(printTask);
    sensor_print_rate = print_rate;
}

void SensorUpdater::updateTasK()
{
    // Check if needs to send a message or updates its data
    if (enable_all_messages and enable_sensor_feedback)
    {
        // Check if needs to update data from sensor
        if (!sensor_data_updated and
            (task_time - last_sensor_time_stamp >= sensor_print_rate / 2))
        {
            update_data_task();
            sensor_data_updated = true;
        }

        // Checks if needs to print data message
        if (task_time - last_sensor_time_stamp >= sensor_print_rate)
        {
            print_data_task();
            Serial.print("\n\r");
            Serial.flush();
            last_sensor_time_stamp = task_time;
            sensor_data_updated = false;
        }
    }
}

void SensorUpdater::updateData()
{
    update_data_task();
}

void SensorUpdater::printData()
{
    print_data_task();
}

bool SensorUpdater::isSensorFeedbackEnabled()
{
    return enable_sensor_feedback;
}

void SensorUpdater::attachPrintTask(void (*function)(void))
{
    print_data_task = function;
}

void SensorUpdater::attachUpdateSensorTask(void (*function)(void))
{
    update_data_task = function;
}

void SensorUpdater::enableSensorFeedback(bool enable)
{
    enable_sensor_feedback = enable;
}

void SensorUpdater::updateTime()
{
    task_time = millis();
}

void SensorUpdater::enableAllMessages(bool enable)
{
    enable_all_messages = enable;
}