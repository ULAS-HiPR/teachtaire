#pragma once
#include <atomic>
#include <cstdint>
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
#include <math.h>
#include <cstdio>
#include <cstdint>
#include <data.h>
#include <Radio/Radio.h>
#include <GNSS/GNSS.h>

#define TELEM_DELAY_MS 100
// this is actully for airbrakes
//180 = in
//0 = out

namespace task{
class Telem_Task {
    public:
        Telem_Task(Radio &radio, GNSS &gnss, osMessageQueueId_t can_queue, osMessageQueueId_t logger_queue)
            : radio_(radio), gnss_(gnss), can_queue_(can_queue), logger_queue_(logger_queue), taskHandle_(nullptr){};
        void run();

    private:
        Radio &radio_;
        GNSS &gnss_;
        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;

        void StartTelemTask();
        static void StartTelemTaskEntry(void *argument);

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "TelemTask",
            0,
            nullptr,
            0,
            nullptr,
            512,        // 512 byte stack
            osPriorityHigh,
            0,
            0
        };
    };

}

