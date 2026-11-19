#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../protocol_frames.h"

namespace aegis::comms {

class SerialCRCBridge {
public:
    static uint16_t calculate_crc16(const uint8_t* data, size_t length);
    static bool verify_frame(const uint8_t* buffer, size_t length);

    static size_t pack_frame(MsgType type, uint16_t seq, uint64_t t_cap_us,
                             const void* payload, uint8_t payload_len, uint8_t* out_buf);

    static size_t pack_telemetry(const SensorFrame50Hz& sensor_data, uint16_t seq,
                                 uint64_t t_cap_us, uint8_t* out_buf);

    // Maquina de framing nao-bloqueante byte-a-byte para RX Host -> MCU
    struct RxParser {
        enum State : uint8_t { WAIT_AA = 0, WAIT_55, READ_HDR, READ_BODY };
        State   state = WAIT_AA;
        uint8_t buf[300] = {};
        size_t  idx = 0;
        size_t  expected_len = 0;

        bool feed_byte(uint8_t b, MsgType& out_type, const uint8_t*& out_payload, uint8_t& out_len);
        void reset() { state = WAIT_AA; idx = 0; expected_len = 0; }
    };
};

} // namespace aegis::comms
