#include "telem_task.h"

namespace task {

void Telem_Task::run() {
    taskHandle_ = osThreadNew(&Telem_Task::StartTelemTaskEntry,
                              this,
                              &task_attributes);
}

void Telem_Task::StartTelemTaskEntry(void *argument) {
    auto *self = static_cast<Telem_Task*>(argument);
    if (self) {
        self->StartTelemTask();
    }
}

void Telem_Task::StartTelemTask() {


    osDelay(1000);

    flight_data flight_data_in;
    gps_data gps_data_out;

    for (;;) {
        osStatus_t status;
        status = osMessageQueueGet(can_queue_, &flight_data_in, NULL, 10U);   // wait for message

        //read gnss
        //send telem

        osMessageQueuePut(logger_queue_, &gps_data_out, 0, 0);
    }
        osDelay(TELEM_DELAY_MS);  
    }
}
