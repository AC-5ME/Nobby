#ifndef UVC_ARDUINO_H
#define UVC_ARDUINO_H

#include <stdint.h>

// --- Protocol Constants ---
#define UCP_SOM 0x7E
#define UCP_EOM 0x7E
#define UCP_MAX_PAYLOAD 16

// --- UCP Frame Structure (no bitfields, unions, or packed attributes) ---
typedef struct {
    uint8_t start;                    // Frame start marker (0x7E)
    uint8_t len;                      // Length: type + payload
    uint8_t type;                     // Message type/ID
    uint8_t payload[UCP_MAX_PAYLOAD]; // Payload (interpreted by type)
    uint16_t crc;                     // CRC-CCITT
    uint8_t end;                      // Frame end marker (0x7E)
} ucp_frame_t;

// --- Example Message Type Definitions ---
#define UCP_MSG_SET_SQUAWK 0x10
#define UCP_MSG_SET_MODE   0x11
#define UCP_MSG_IDENT      0x12
#define UCP_MSG_STATUS_REQ 0x80
#define UCP_MSG_STATUS     0x81
// Add others as needed

// --- CRC-CCITT (XModem) Calculation ---
static inline uint16_t ucp_crc16_ccitt(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// --- Helpers for Manual Bit Manipulation (example for status byte) ---
#define UCP_STATUS_READY_MASK 0x01
#define UCP_STATUS_FAIL_MASK  0x02

static inline uint8_t ucp_status_is_ready(uint8_t status_byte) {
    return (status_byte & UCP_STATUS_READY_MASK) != 0;
}
static inline uint8_t ucp_status_is_fail(uint8_t status_byte) {
    return (status_byte & UCP_STATUS_FAIL_MASK) != 0;
}

// --- Example: Accessing Multi-byte Values in Payload ---
static inline uint16_t ucp_payload_get_u16(const uint8_t *payload, uint8_t idx) {
    return (uint16_t)payload[idx] | ((uint16_t)payload[idx+1] << 8);
}
static inline void ucp_payload_set_u16(uint8_t *payload, uint8_t idx, uint16_t value) {
    payload[idx]   = value & 0xFF;
    payload[idx+1] = (value >> 8) & 0xFF;
}

// Add other helpers for protocol features as needed

#endif // UVC_ARDUINO_H