#include "CAN_task.h"

namespace task {

void CAN_task::run() {
    taskHandle_ = osThreadNew(&CAN_task::StartCANEntry,
                              this,
                              &task_attributes);
}

void CAN_task::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN_task*>(argument);

    if (self) {
        self->StartCAN();
    }
}

void CAN_task::StartCAN() {

    printf("CAN task started\n");

    CAN_Frame rx_frame;

    flight_data shared_data{};
    gps_data logger_data{};

    for (;;) {
        if (canbus_.receive(&rx_frame)) {
            switch (rx_frame.id) {
                case CAN_ID_KALMANN: {
                    if (rx_frame.dlc != sizeof(KALMANN_Payload)) {
                        break;
                    }

                    KALMANN_Payload kalman{};
                    unpack_frame(rx_frame, kalman);

                    shared_data.prediction.altitude =  kalman.altitude_m;
                    shared_data.prediction.velocity = kalman.vspeed_cms / 100.0f;
                    shared_data.prediction.acceleration = kalman.accleration / 100.0f;

                    break;
                }
                case CAN_ID_FLIGHT_STATE: {
                    if (rx_frame.dlc != sizeof(FLIGHT_STATE_Payload)) {
                        break;
                    }

                    FLIGHT_STATE_Payload state{};
                    unpack_frame(rx_frame, state);

                    shared_data.state = state.state;

                    break;
                }

                default:
                    break;
            }
        }
        // --- IGNORE THIS TEST CODE ---/
            shared_data.prediction.altitude = 100.0f;
            shared_data.prediction.velocity = 50.0f;
            shared_data.prediction.acceleration = -9.8f;
            shared_data.state = 1;
            osMessageQueuePut(reciver_queue_, &shared_data, 0, 10U);
        //}

        //GNSS CAN Frame doesnt exisit
        //if (osMessageQueueGet(sender_queue_, &logger_data, 0, 0U) == osOK) {
        //    //temp
        //    CAN_Frame dbg;
        //    dbg.id  = 0x123;
        //    dbg.dlc = 8;
        //    
        //    dbg.data[0] = 0xDE;
        //    dbg.data[1] = 0xAD;
        //    dbg.data[2] = 0xBE;
        //    dbg.data[3] = 0xEF;
        //    dbg.data[4] = 0x01;
        //    dbg.data[5] = 0x02;
        //    dbg.data[6] = 0x03;
        //    dbg.data[7] = 0x04;
        //    canbus_.send(&dbg);
        //}

        //// always send hearbeat
        //HEARTBEAT_Payload hb{};
        //hb.node_id = NODE_TEACHTAIRE;
        //hb.state   = 1;
        //hb.err     = 0;
        //hb.uptime_s = 0;
        //CAN_Frame hb_frame =
        //pack_frame(CAN_ID_HEARTBEAT, hb);
        //canbus_.send(&hb_frame);

        CAN_Frame dbg;
        dbg.id  = 0x123;
        dbg.dlc = 8;

        dbg.data[0] = 0xDE;
        dbg.data[1] = 0xAD;
        dbg.data[2] = 0xBE;
        dbg.data[3] = 0xEF;
        dbg.data[4] = 0x01;
        dbg.data[5] = 0x02;
        dbg.data[6] = 0x03;
        dbg.data[7] = 0x04;
        canbus_.send(&dbg);
        
        osDelay(CAN_DELAY_MS);
    }
}

}