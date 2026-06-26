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

#define CAN_DELAY_MS 200
#define CAN_HEARTBEAT_PERIOD_MS 1000U
#define CAN_FLIGHT_TX_MIN_PERIOD_MS 20U
#define CAN_BUS_RECOVERY_PERIOD_MS 250U
#define CAN_NODE_TIMEOUT_MS 5000U
#define CAN_AUTO_RECOVER_BUS_OFF 1
#define CAN_TX_RETRY_QUEUE_LEN 16U
#define CAN_TX_DRAIN_BUDGET_PER_LOOP 3U
#define CAN_MAX_TRACKED_NODES 8U

namespace task{
class CAN_task {
    public:
        CAN_task(CAN_Handler& canbus, osMessageQueueId_t sender_queue,
                 osMessageQueueId_t reciver_queue, uint8_t node_id = NODE_CROI) :
            canbus_(canbus),
            sender_queue_(sender_queue),
            reciver_queue_(reciver_queue),
            taskHandle_(nullptr),
            node_id_(node_id) {};
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        bool process_rx_frame(const CAN_Frame& frame, flight_data& shared_data);
        void send_gps_data(const gps_data& data);
        void send_heartbeat(uint32_t now_ms);
        void service_bus_health(uint32_t now_ms);
        void expire_node_status(uint32_t now_ms);
        void flush_tx_queue();
        bool queue_tx_frame(const CAN_Frame& frame);
        void record_heartbeat(const HEARTBEAT_Payload& heartbeat, uint32_t now_ms);
        bool send_frame(CAN_Frame& frame);

        CAN_Handler& canbus_;
        osMessageQueueId_t sender_queue_;
        osMessageQueueId_t reciver_queue_;

        osThreadId_t taskHandle_;
        uint8_t node_id_;
        uint32_t last_heartbeat_ms_{0U};
        uint32_t last_flight_tx_ms_{0U};
        uint32_t last_bus_recovery_ms_{0U};
        uint8_t flight_state_{0U};
        gps_data pending_outbound_data_{};
        bool has_pending_outbound_data_{false};

        CAN_Frame tx_retry_queue_[CAN_TX_RETRY_QUEUE_LEN]{};
        uint8_t tx_retry_head_{0U};
        uint8_t tx_retry_tail_{0U};
        uint8_t tx_retry_count_{0U};
        uint32_t tx_retry_drops_{0U};
        uint32_t node_timeout_count_{0U};

        struct NodeStatus {
            uint8_t node_id{0U};
            uint8_t state{0U};
            uint8_t err{0U};
            uint8_t tx_error_count{0U};
            uint8_t rx_error_count{0U};
            uint8_t tx_queue_depth{0U};
            uint16_t uptime_s{0U};
            uint32_t last_seen_ms{0U};
            uint32_t rx_count{0U};
            bool active{false};
        };

        NodeStatus nodes_[CAN_MAX_TRACKED_NODES]{};

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            nullptr,
            0,
            nullptr,
            1024,
            osPriorityNormal,
            0,
            0
        };
    };

}