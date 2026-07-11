#pragma once

#include <cstddef>
#include <cstdint>

namespace ogma_radio {

constexpr uint8_t kMagic0 = 'O';
constexpr uint8_t kMagic1 = 'G';
constexpr uint8_t kVersion = 1U;
constexpr std::size_t kHeaderSize = 12U;
constexpr std::size_t kCrcSize = 2U;
constexpr std::size_t kCanRecordSize = 11U;
constexpr std::size_t kMaxCanRecords = 4U;

enum class PacketType : uint8_t {
    CanBundle = 1U,
    Gps = 2U,
    Test = 0x7FU,
};

struct RawCanRecord {
    uint16_t id{0U};
    uint8_t dlc{0U};
    uint8_t data[8]{};
};

inline void write_u16_le(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

inline void write_u32_le(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    out[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    out[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

inline uint16_t crc16_ccitt(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0U; index < length; ++index) {
        crc ^= static_cast<uint16_t>(data[index]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

inline std::size_t write_header(uint8_t* out,
                                std::size_t max_length,
                                PacketType type,
                                uint16_t sequence,
                                uint32_t uptime_ms,
                                uint8_t record_count,
                                uint8_t flags) {
    if (out == nullptr || max_length < kHeaderSize + kCrcSize) {
        return 0U;
    }
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = static_cast<uint8_t>(type);
    write_u16_le(&out[4], sequence);
    write_u32_le(&out[6], uptime_ms);
    out[10] = record_count;
    out[11] = flags;
    return kHeaderSize;
}

inline std::size_t finish_packet(uint8_t* out,
                                 std::size_t max_length,
                                 std::size_t content_length) {
    if (out == nullptr || content_length + kCrcSize > max_length) {
        return 0U;
    }
    const uint16_t crc = crc16_ccitt(out, content_length);
    write_u16_le(&out[content_length], crc);
    return content_length + kCrcSize;
}

inline std::size_t build_can_bundle(uint8_t* out,
                                    std::size_t max_length,
                                    uint16_t sequence,
                                    uint32_t uptime_ms,
                                    const RawCanRecord* records,
                                    std::size_t record_count,
                                    uint8_t flags = 0U) {
    if (records == nullptr || record_count == 0U || record_count > kMaxCanRecords) {
        return 0U;
    }
    const std::size_t needed = kHeaderSize + record_count * kCanRecordSize + kCrcSize;
    if (out == nullptr || needed > max_length) {
        return 0U;
    }
    std::size_t offset = write_header(out, max_length, PacketType::CanBundle,
                                      sequence, uptime_ms,
                                      static_cast<uint8_t>(record_count), flags);
    for (std::size_t index = 0U; index < record_count; ++index) {
        const RawCanRecord& record = records[index];
        if (record.id > 0x7FFU || record.dlc > 8U) {
            return 0U;
        }
        write_u16_le(&out[offset], record.id);
        out[offset + 2U] = record.dlc;
        for (uint8_t byte = 0U; byte < 8U; ++byte) {
            out[offset + 3U + byte] = record.data[byte];
        }
        offset += kCanRecordSize;
    }
    return finish_packet(out, max_length, offset);
}

inline std::size_t build_gps_packet(uint8_t* out,
                                    std::size_t max_length,
                                    uint16_t sequence,
                                    uint32_t uptime_ms,
                                    const uint8_t* gps_payload,
                                    std::size_t gps_length,
                                    uint8_t flags = 0U) {
    const std::size_t needed = kHeaderSize + gps_length + kCrcSize;
    if (out == nullptr || gps_payload == nullptr || gps_length == 0U || needed > max_length) {
        return 0U;
    }
    std::size_t offset = write_header(out, max_length, PacketType::Gps,
                                      sequence, uptime_ms, 1U, flags);
    for (std::size_t index = 0U; index < gps_length; ++index) {
        out[offset + index] = gps_payload[index];
    }
    offset += gps_length;
    return finish_packet(out, max_length, offset);
}

inline std::size_t build_test_packet(uint8_t* out,
                                     std::size_t max_length,
                                     uint16_t sequence,
                                     uint32_t uptime_ms,
                                     uint32_t counter) {
    if (out == nullptr || max_length < kHeaderSize + 4U + kCrcSize) {
        return 0U;
    }
    std::size_t offset = write_header(out, max_length, PacketType::Test,
                                      sequence, uptime_ms, 1U, 0U);
    write_u32_le(&out[offset], counter);
    return finish_packet(out, max_length, offset + 4U);
}

} // namespace ogma_radio
