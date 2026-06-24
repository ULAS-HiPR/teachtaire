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
    flight_data logger_data{};

    for (;;) {
        if (canbus_.receive(&rx_frame)) {
            switch (rx_frame.id) {
                case CAN_ID_IMU_ACCEL: {

                    if (rx_frame.dlc != sizeof(IMU_ACCEL_Payload)) {
                        break;
                    }

                    IMU_ACCEL_Payload accel{};
                    unpack_frame(rx_frame, accel);

                    shared_data.core_data.imu.acceleration.x = accel.ax / 100.0f;
                    shared_data.core_data.imu.acceleration.y = accel.ay / 100.0f;
                    shared_data.core_data.imu.acceleration.z = accel.az / 100.0f;

                    break;
                }

                case CAN_ID_IMU_GYRO: {

                    if (rx_frame.dlc != sizeof(IMU_GYRO_Payload)) {
                        break;
                    }

                    IMU_GYRO_Payload gyro{};
                    unpack_frame(rx_frame, gyro);

                    shared_data.core_data.imu.gyro.x = gyro.gx / 100.0f;
                    shared_data.core_data.imu.gyro.y = gyro.gy / 100.0f;
                    shared_data.core_data.imu.gyro.z = gyro.gz / 100.0f;

                    break;
                }

                case CAN_ID_BARO: {

                    if (rx_frame.dlc != sizeof(BARO_Payload)) {
                        break;
                    }

                    BARO_Payload baro{};
                    unpack_frame(rx_frame, baro);

                    shared_data.core_data.barometer.pressure = static_cast<float>(baro.pressure);
                    shared_data.core_data.barometer.temperature =  baro.temp / 100.0f;

                    break;
                }
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
            shared_data.core_data.imu.gyro.x = 100.0f; 
            shared_data.core_data.imu.gyro.y = 0.0f;
            shared_data.core_data.imu.gyro.z = 0.0f;
            shared_data.core_data.barometer.pressure = 101325.0f;
            shared_data.core_data.barometer.temperature = 20.0f;
            shared_data.prediction.altitude = 100.0f;
            shared_data.prediction.velocity = 50.0f;
            shared_data.prediction.acceleration = -9.8f;
            shared_data.state = 1;
            osMessageQueuePut(reciver_queue_, &shared_data, 0, 10U);
        //}

        if (osMessageQueueGet(sender_queue_, &logger_data, 0, 0U) == osOK) {
            CAN_Frame imu_accel_frame;
            CAN_Frame imu_gyro_frame;
            CAN_Frame baro_frame;
            CAN_Frame kalman_frame;
            CAN_Frame state_frame;

            IMU_ACCEL_Payload accel_payload{
                static_cast<int16_t>(logger_data.core_data.imu.acceleration.x * 100.0f),
                static_cast<int16_t>(logger_data.core_data.imu.acceleration.y * 100.0f),
                static_cast<int16_t>(logger_data.core_data.imu.acceleration.z * 100.0f),
                static_cast<uint16_t>(logger_data.core_data.time % 65536)
            };

            IMU_GYRO_Payload gyro_payload{
                static_cast<int16_t>(logger_data.core_data.imu.gyro.x * 100.0f),
                static_cast<int16_t>(logger_data.core_data.imu.gyro.y * 100.0f),
                static_cast<int16_t>(logger_data.core_data.imu.gyro.z * 100.0f),
                static_cast<uint16_t>(logger_data.core_data.time % 65536)
            };

            BARO_Payload baro_payload{
                static_cast<uint32_t>(logger_data.core_data.barometer.pressure),
                static_cast<int16_t>(logger_data.core_data.barometer.temperature * 100.0f),
                static_cast<uint16_t>(logger_data.core_data.time % 65536)
            };

            KALMANN_Payload kalman_payload{
                static_cast<int16_t>(logger_data.prediction.altitude),
                static_cast<int16_t>(logger_data.prediction.velocity * 100.0f),
                static_cast<int16_t>(logger_data.prediction.acceleration * 100.0f),
                static_cast<uint16_t>(logger_data.core_data.time % 65536)
            };

            FLIGHT_STATE_Payload state_payload{
                static_cast<uint8_t>(logger_data.state),
                0,
                static_cast<uint16_t>(logger_data.core_data.time % 65536)
            };

            imu_accel_frame = pack_frame(CAN_ID_IMU_ACCEL, accel_payload);
            imu_gyro_frame = pack_frame(CAN_ID_IMU_GYRO, gyro_payload);
            baro_frame = pack_frame(CAN_ID_BARO, baro_payload);
            kalman_frame = pack_frame(CAN_ID_KALMANN, kalman_payload);
            state_frame = pack_frame(CAN_ID_FLIGHT_STATE, state_payload);
            canbus_.send(&imu_accel_frame);
            canbus_.send(&imu_gyro_frame);
            canbus_.send(&baro_frame);
            canbus_.send(&kalman_frame);
            canbus_.send(&state_frame);
        }

        //// always send hearbeat
        //HEARTBEAT_Payload hb{};
        //hb.node_id = NODE_CROI;
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

        
        

        if (canbus_.receive(&rx_frame))
            printf("RX OK\n");
        
        osDelay(CAN_DELAY_MS);
    }
}

}