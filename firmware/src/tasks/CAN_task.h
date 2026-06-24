#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "platform/hal_time.h"
#include "cmsis_os.h"
#include <cstdio>
#include <data.h>

#include <CAN/CAN_Handler.h>
#include <CAN/CAN_Frames.h>

#define CAN_DELAY_MS 100

namespace task{
class CAN_task {
    public:
        CAN_task(CAN_Handler& canbus, osMessageQueueId_t sender_queue, osMessageQueueId_t reciver_queue) :
            canbus_(canbus), sender_queue_(sender_queue), reciver_queue_(reciver_queue), taskHandle_(nullptr) {};
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        char parse_message(char msg);

        CAN_Handler& canbus_;
        osMessageQueueId_t sender_queue_;
        osMessageQueueId_t reciver_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            nullptr,
            0,
            nullptr,
            512,       
            osPriorityNormal,
            0,
            0
        };
    };

}

