#include "CAN_task.h"

namespace task {

namespace {

void put_drop_oldest(osMessageQueueId_t queue, const flight_data& data) {
    if (osMessageQueuePut(queue, &data, 0U, 0U) == osOK) {
        return;
    }

    flight_data discarded{};
    (void)osMessageQueueGet(queue, &discarded, nullptr, 0U);
    (void)osMessageQueuePut(queue, &data, 0U, 0U);
}

}

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
    flight_data shared_data{};
    gps_data outbound_data{};
    last_heartbeat_ms_ = HAL_GetTick();
    last_flight_tx_ms_ = last_heartbeat_ms_;
    last_bus_recovery_ms_ = last_heartbeat_ms_;

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();
        service_bus_health(now_ms);
        expire_node_status(now_ms);
        flush_tx_queue();

        bool received_update = false;
        CAN_Frame rx_frame{};
        while (canbus_.receive(&rx_frame)) {
            received_update |= process_rx_frame(rx_frame, shared_data);
        }

        if (received_update) {
            put_drop_oldest(reciver_queue_, shared_data);
        }

        while (osMessageQueueGet(sender_queue_, &outbound_data, nullptr, 0U) == osOK) {
            pending_outbound_data_ = outbound_data;
            has_pending_outbound_data_ = true;
        }

        if (has_pending_outbound_data_ &&
            (now_ms - last_flight_tx_ms_) >= CAN_FLIGHT_TX_MIN_PERIOD_MS) {
            send_gps_data(pending_outbound_data_);
            has_pending_outbound_data_ = false;
            last_flight_tx_ms_ = now_ms;
        }

        if ((now_ms - last_heartbeat_ms_) >= CAN_HEARTBEAT_PERIOD_MS) {
            send_heartbeat(now_ms);
            last_heartbeat_ms_ = now_ms;
        }

        osDelay(CAN_DELAY_MS);
    }
}

bool CAN_task::process_rx_frame(const CAN_Frame& frame, flight_data& shared_data) {
    if (CAN_ID_IS_HEARTBEAT(frame.id)) {
        HEARTBEAT_Payload payload{};
        if (!try_unpack_frame(frame, payload)) {
            return false;
        }
        record_heartbeat(payload, HAL_GetTick());
        return false;
    }

    switch (frame.id) {
        case CAN_ID_KALMANN: {
            KALMANN_Payload payload{};
            if (!try_unpack_frame(frame, payload)) {
                return false;
            }
            shared_data.prediction.acceleration = payload.accleration / 100.0f;
            shared_data.prediction.altitude = payload.altitude_m ;
            shared_data.prediction.velocity = payload.vspeed_cms / 100.0f;
            shared_data.core_data.time = payload.timestamp_ms;
            return true;
        }

        case CAN_ID_FLIGHT_STATE: {
            FLIGHT_STATE_Payload payload{};
            if (!try_unpack_frame(frame, payload)) {
                return false;
            }
            shared_data.state = payload.state;
            shared_data.time = payload.timestamp_ms;
            flight_state_ = payload.state;
            return true;
        }

        default:
            return false;
    }
}

void CAN_task::send_gps_data(const gps_data& data) {
    int32_t lat = gps_encode(data.latitude);
    int32_t lon = gps_encode(data.longitude);

    CAN_Frame gps_frame = pack_gps(
        CAN_ID_GPS,
        lat,
        lon,
        data.satellites,
        0U
    );

    send_frame(gps_frame);
}

void CAN_task::send_heartbeat(uint32_t now_ms) {
    uint8_t err = 0U;
    if (canbus_.is_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (canbus_.error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (tx_retry_drops_ != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }
    if (node_timeout_count_ != 0U) {
        err |= CAN_HEARTBEAT_ERR_NODE_TIMEOUT;
    }

    HEARTBEAT_Payload payload{
        node_id_,
        flight_state_,
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU)
    };
    CAN_Frame frame = pack_frame(CAN_ID_HEARTBEAT, payload);
    send_frame(frame);
}

void CAN_task::service_bus_health(uint32_t now_ms) {
#if CAN_AUTO_RECOVER_BUS_OFF
    if (!canbus_.is_bus_off()) {
        return;
    }

    if ((now_ms - last_bus_recovery_ms_) < CAN_BUS_RECOVERY_PERIOD_MS) {
        return;
    }

    (void)canbus_.recover_from_bus_off();
    last_bus_recovery_ms_ = now_ms;
#else
    (void)now_ms;
#endif
}

void CAN_task::flush_tx_queue() {
    for (uint8_t sent = 0U;
         sent < CAN_TX_DRAIN_BUDGET_PER_LOOP && tx_retry_count_ > 0U;
         ++sent) {
        CAN_Frame& frame = tx_retry_queue_[tx_retry_head_];
        if (!canbus_.send(&frame)) {
            return;
        }

        tx_retry_head_ = static_cast<uint8_t>(
            (tx_retry_head_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
        --tx_retry_count_;
    }
}

bool CAN_task::queue_tx_frame(const CAN_Frame& frame) {
    if (tx_retry_count_ >= CAN_TX_RETRY_QUEUE_LEN) {
        tx_retry_head_ = static_cast<uint8_t>(
            (tx_retry_head_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
        --tx_retry_count_;
        ++tx_retry_drops_;
    }

    tx_retry_queue_[tx_retry_tail_] = frame;
    tx_retry_tail_ = static_cast<uint8_t>(
        (tx_retry_tail_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
    ++tx_retry_count_;
    return true;
}

void CAN_task::record_heartbeat(const HEARTBEAT_Payload& heartbeat,
                                uint32_t now_ms) {
    if (heartbeat.node_id == node_id_ || heartbeat.node_id == 0U) {
        return;
    }

    for (NodeStatus& node : nodes_) {
        if (node.active && node.node_id == heartbeat.node_id) {
            node.state = heartbeat.state;
            node.err = heartbeat.err;
            node.tx_error_count = 0U;
            node.rx_error_count = 0U;
            node.tx_queue_depth = 0U;
            node.uptime_s = heartbeat.uptime_s;
            node.last_seen_ms = now_ms;
            ++node.rx_count;
            return;
        }
    }

    for (NodeStatus& node : nodes_) {
        if (!node.active) {
            node.active = true;
            node.node_id = heartbeat.node_id;
            node.state = heartbeat.state;
            node.err = heartbeat.err;
            node.tx_error_count = 0U;
            node.rx_error_count = 0U;
            node.tx_queue_depth = 0U;
            node.uptime_s = heartbeat.uptime_s;
            node.last_seen_ms = now_ms;
            node.rx_count = 1U;
            return;
        }
    }
}

void CAN_task::expire_node_status(uint32_t now_ms) {
    for (NodeStatus& node : nodes_) {
        if (!node.active) {
            continue;
        }

        if ((now_ms - node.last_seen_ms) >= CAN_NODE_TIMEOUT_MS) {
            node.active = false;
            node.err = 1U;
            ++node_timeout_count_;
        }
    }
}

bool CAN_task::send_frame(CAN_Frame& frame) {
    if (!canbus_.send(&frame)) {
        queue_tx_frame(frame);
    }

    if (canbus_.send(&frame)) {
        return true;
    }

    return queue_tx_frame(frame);
}

}