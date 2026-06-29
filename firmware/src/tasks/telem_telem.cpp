#include "telem_task.h"
#include <cstdio>

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
    flight_data flight_data_in{};
    gps_data gps_data_out{};

    for (;;) {
        osStatus_t status;
        //status = osMessageQueueGet(can_queue_, &flight_data_in, NULL, 10U);   // wait for message

        uint8_t sending_buffer[32] = {0};

        bool gps_ok = gnss_.update(&gps_data_out);
        if (gps_ok) {
            printf("gps");
        }
       
        std::size_t len = pack_gps(
            gps_data_out,
            sending_buffer,
            gps_ok
        );

        bool connected = gnss_.poll_navigation_satellites();

        radio_.send(sending_buffer, len);
        osMessageQueuePut(logger_queue_, &gps_data_out, 0, 0);
    
        osDelay(TELEM_DELAY_MS);  
    }
}
}