#include "serial_crc_bridge.h"
#include <string.h>

namespace aegis::comms {

uint16_t SerialCRCBridge::calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t j = 0; j < 8; ++j) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool SerialCRCBridge::verify_frame(const uint8_t* buffer, size_t length) {
    if (length < sizeof(FrameHeader) + 2U) return false;
    FrameHeader hdr{};
    memcpy(&hdr, buffer, sizeof(FrameHeader));
    if (hdr.sync != 0xAA55U || hdr.version != 0x01U) return false;

    const size_t expected_total = sizeof(FrameHeader) + hdr.payload_len + 2U;
    if (length < expected_total) return false;

    const uint16_t calc_crc = calculate_crc16(buffer + 2, sizeof(FrameHeader) - 2U + hdr.payload_len);
    const uint16_t rx_crc = static_cast<uint16_t>((static_cast<uint16_t>(buffer[expected_total - 2]) << 8) |
                                                   buffer[expected_total - 1]);
    return (calc_crc == rx_crc);
}

size_t SerialCRCBridge::pack_frame(MsgType type, uint16_t seq, uint64_t t_cap_us,
                                   const void* payload, uint8_t payload_len, uint8_t* out_buf) {
    FrameHeader hdr{};
    hdr.sync = 0xAA55U;
    hdr.version = 0x01U;
    hdr.msg_type = static_cast<uint8_t>(type);
    hdr.seq_num = seq;
    hdr.t_cap_us = t_cap_us;
    hdr.payload_len = payload_len;

    memcpy(out_buf, &hdr, sizeof(FrameHeader));
    if (payload != nullptr && payload_len > 0U) {
        memcpy(out_buf + sizeof(FrameHeader), payload, payload_len);
    }

    const uint16_t crc = calculate_crc16(out_buf + 2, sizeof(FrameHeader) - 2U + payload_len);
    out_buf[sizeof(FrameHeader) + payload_len]      = static_cast<uint8_t>(crc >> 8);
    out_buf[sizeof(FrameHeader) + payload_len + 1U] = static_cast<uint8_t>(crc & 0xFFU);
    return sizeof(FrameHeader) + payload_len + 2U;
}

size_t SerialCRCBridge::pack_telemetry(const SensorFrame50Hz& sensor_data, uint16_t seq,
                                       uint64_t t_cap_us, uint8_t* out_buf) {
    return pack_frame(MsgType::SENSOR_50HZ, seq, t_cap_us, &sensor_data,
                      static_cast<uint8_t>(sizeof(SensorFrame50Hz)), out_buf);
}

bool SerialCRCBridge::RxParser::feed_byte(uint8_t b, MsgType& out_type,
                                          const uint8_t*& out_payload, uint8_t& out_len) {
    switch (state) {
        case WAIT_AA:
            if (b == 0x55U) { // Little-Endian 0xAA55 -> LSB 0x55 primeiro
                buf[0] = b;
                idx = 1;
                state = WAIT_55;
            }
            break;
        case WAIT_55:
            if (b == 0xAAU) {
                buf[1] = b;
                idx = 2;
                state = READ_HDR;
            } else {
                state = WAIT_AA;
            }
            break;
        case READ_HDR:
            buf[idx++] = b;
            if (idx == sizeof(FrameHeader)) {
                const uint8_t plen = buf[sizeof(FrameHeader) - 1U];
                expected_len = sizeof(FrameHeader) + plen + 2U;
                if (expected_len > sizeof(buf)) {
                    reset();
                } else {
                    state = READ_BODY;
                }
            }
            break;
        case READ_BODY:
            buf[idx++] = b;
            if (idx >= expected_len) {
                const bool ok = SerialCRCBridge::verify_frame(buf, expected_len);
                reset();
                if (ok) {
                    out_type = static_cast<MsgType>(buf[3]);
                    out_len = buf[sizeof(FrameHeader) - 1U];
                    out_payload = &buf[sizeof(FrameHeader)];
                    return true;
                }
            }
            break;
    }
    return false;
}

} // namespace aegis::comms
