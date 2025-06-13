#ifndef UVC_H
#define UVC_H

#include <stdint.h>
#include <stdbool.h>

uint16_t Crc16Table[256];

// Transmit message sizes
#define UCV_TX_MAX_PAYLOAD_LENGTH      552
#define UCV_OVERHEAD_LENGTH            3 // Not counting framing bytes
#define UCV_TX_MAX_PACKET_LENGTH       (UCV_TX_MAX_PAYLOAD_LENGTH + UCV_OVERHEAD_LENGTH)
#define UCV_TX_MAX_FRAME_LENGTH        (2 + ((15 * UCV_TX_MAX_PACKET_LENGTH) / 10)) // If every other byte was escaped. C.f. https://stackoverflow.com/q/75914914/4271922

// Receive message sizes
#define UCV_RX_MAX_PAYLOAD_LENGTH      128
#define UCV_RX_MAX_PACKET_LENGTH       (UCV_RX_MAX_PAYLOAD_LENGTH + UCV_OVERHEAD_LENGTH)

enum MAGIC_BYTES {
    FRAME_FLAG_BYTE = 0x7E,
    CONTROL_ESCAPE_BYTE = 0x7D,
    STUFF_BYTE = 0x20,
};

enum packet_ids {
    TPDR_CONFIGURATION     = 0x2B,
    MSG_RQST               = 0x2B,
    HEARTBEAT              = 0x00,
    OWNSHIP_REPORT         = 0x0A,
    GEO_ALT_OWNSHIP_REPORT = 0x0B,
    ID_MESSAGE             = 0x25,
    SENSOR_MESSAGE         = 0x28,
    TPDR_CONTROL           = 0x2D,
    GNSS_DATA              = 0x2E,
    TPDR_STATUS            = 0x2F,
    UAVIONIX_OWM           = 0x75,
};

enum proto_states {
    START,
    UCV_ID,
//    UBX_LEN1,
//    UBX_LEN2,
    UCV_PAYLOAD,
    UCV_CHK1,
    UCV_CHK2,
    FINISHED
};

typedef struct __attribute__((packed)) {
    uint8_t frameFlagBegin;
    uint8_t msgID;
    uint8_t *payload;
    uint8_t frameCheck[2];
    uint8_t frameFlagEnd;
} UVC_packet;


/*
 * Transponder Configuration Messages should be sent only as required to set or update the
 * transponders version of the information contained within this message. Valid receipt of
 * this message will cause the transponder to write the new data into non-volatile memory
 *  and all data will persist through a power cycle.
 *   [Host to device: Sent to device as required for configuration]
 *   [Device to host: Output by device, sent upon request]
 */
typedef struct __attribute__((packed)) {
    uint8_t msgID;
    uint8_t msgVersion;
    uint8_t icaoAddress_msbFirst[3];  // MSB first
    union {
        struct __attribute__((packed)){
            uint8_t acMaxSpeed            : 3;  // 0:2
            uint8_t baroAltitudeSource    : 1;  // 3
            uint8_t systemDesignAssurance : 2;  // 4:5
            uint8_t sourceLevelIntegrity  : 2;  // 6:7
        };
        uint8_t val;
    } flag1;
    union {
        struct __attribute__((packed)){
            uint8_t acLengthAndWidth : 4;  // 0:3
            uint8_t adsbInCapability : 2;  // 4:5
            uint8_t testMode         : 2;  // 6:7
        };
        uint8_t val;
    } flag2;
    union {
        struct __attribute__((packed)){
            uint8_t gnssAntennaLongitudialOffset : 4;  // 0:3
            uint8_t gnssAntennaLateralOffset     : 4;  // 4:7
        };
        uint8_t val;
    } flag3;
    uint8_t acRegistration[8];  // ASCII string [A-Z, 0-9] only, e.g. "N8644B ". Trailing spaces (0x20) only.
    uint16_t acStallSpeed;  // 0 if equipped with on-ground sensor
    uint8_t acEmitterType;
    union {
        struct __attribute__((packed)){
            uint8_t serialPortBaudRate    : 4;  // 0:3
            uint8_t defaultModeAReplyMode : 1;  // 4
            uint8_t defaultModeCReplyMode : 1;  // 5
            uint8_t defaultModeSReplyMode : 1;  // 6
            uint8_t default1090esTxMode   : 1;  // 7
        };
        uint8_t val;
    } flag4;
    uint16_t defaultModeASquawkCode;  // typically 1200 [0x04B0] for VFR
    uint32_t validityBitmask;  // On host to device, this bit controls whether valid data has been provided and should be applied
    union {
        struct __attribute__((packed)){
            uint8_t reserved                     : 7;  // 0:6
            uint8_t barometricAltitudeResolution : 8;  // 7
        };
        uint8_t val;
    } flag5;
    uint16_t serialInputProtocol;
    uint16_t serialOutputProtocol;
} TpdrConfigurationV0x04;


/**
 * When a Message Request message is sent to the transponder, the transponder
 * will respond with the requested message in reply. This allows a method for
 * the host to request the transponder send a message.
 *  [Host to device: Sent by host to request message]
 */
typedef struct __attribute__((packed)) {
    uint8_t msgID;
    uint8_t msgVersion;
    uint8_t requestedMsgId;
} MessageRequestV0x02;

/**
 * The Control message contains all data to control the immediate status of the
 * transponder along with configuration fields that are expected to change as a
 * normal course of a flight. Control messages must be sent with a one second
 * period regardless of data status or updates.
 *  [Host to device: Sent to device once per second]
 */
typedef struct __attribute__((packed)) {
    const uint8_t msgID;
    const uint8_t msgVersion;
    union {
        struct __attribute__((packed)) {
            uint8_t externalBaroCrossChecked : 1;  // 0
            uint8_t acAirGroundState         : 2;  // 1:2
            uint8_t identButtonActive        : 1;  // 3
            uint8_t modeAReplyEnable         : 1;  // 4
            uint8_t modeCReplyEnable         : 1;  // 5
            uint8_t modeSReplyEnable         : 1;  // 6
            uint8_t _1090esTxEnabled         : 1;  // 7
        };
        uint8_t val;
    } flag1;
    int32_t externalBaroPressureAltitude;  // relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). If unknown set to INT32_MAX
    uint16_t modeASquawk;  // (typically 1200 [0x04B0] for VFR)
    uint8_t emergencyStateStatus;
    uint8_t flightIdentification[8];  // 8 ASCII characters, ‘0’ through ‘9’, ‘A’ through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. e.g. "UA123 ". Aircraft Registration is transmitted if Flight ID is set to all spaces.
} ControlV0x01;


/**
 * The Heartbeat message provides real-time indications of the status and operation of the
 * transponder. The message will be transmitted with a period of one second for the UCP
 * protocol.
 *  [Device to host: Sent to host once per second]
 */
typedef struct __attribute__((packed)) {
    const uint8_t msgID;
    union {
        struct __attribute__((packed)) {
            uint8_t deviceInitialized        : 1;  // 0,  0 = Device is initialized
            uint8_t failureGnssDataFrequency : 1;  // 1
            uint8_t reserved                 : 2;  // 2:3
            uint8_t addressType              : 1;  // 4
            uint8_t ident                    : 1;  // 5
            uint8_t maintenanceRequired      : 1;  // 6
            uint8_t gnssPositionValid        : 1;  // 7
        };
        uint8_t val;
    } flag1;
    union {
        struct __attribute__((packed)){
            uint8_t utcOk                   : 1;  // 0,  1 = UTC timing is valid
            uint8_t fialureGnssUnvavailable : 1;  // 1
            uint8_t fialureGnssNo3dFix      : 1;  // 2
            uint8_t failureBroadcastMonitor : 1;  // 3
            uint8_t failureTxSystem         : 1;  // 4
            uint8_t reserved                : 2;  // 5:6
            uint16_t timeStamp              : 1;  // 17th bit of the timestamp (See below)
        };
        uint8_t val;
    } flag2;
    uint16_t timeStamp;  // Seconds since 0000Z, lower 16 bits
    uint16_t reserved;
} HeartbeatV0x00;




/**
 * @brief crcInit Initializes the CRC lookup table
 */
void crcInit()
{
    uint16_t crc;

    for (uint16_t i = 0; i < 256; i++) {
        crc = (i << 8);
        for (uint8_t bitctr = 0; bitctr < 8; bitctr++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }

        Crc16Table[i] = crc;
    }
}

/**
 * @brief crcCompute return CRC of the block
 * @param block Starting address of message
 * @param length Length of message
 * @return
 */
unsigned int crcCompute(uint8_t *block, uint32_t length)
{
    uint32_t i;
    uint16_t crc = 0;

    for (i = 0; i < length; i++)
    {
        crc = Crc16Table[crc >> 8] ^ (crc << 8) ^ block[i];
    }

    return crc;
}


/**
 * @brief marshallUvcMessage
 * @param inputBuf
 * @param outputBuf buffer with max length UCV_TX_MAX_FRAME_LENGTH
 * @param length
 * @return
 */
int marshallUvcMessage(uint8_t inputBuf[UCV_TX_MAX_PAYLOAD_LENGTH], uint8_t outputBuf[UCV_TX_MAX_FRAME_LENGTH], uint32_t length)
{
    // Calculate the CRC
    uint16_t frameCrc = crcCompute(inputBuf, length);

    // Set flag byte in frame buffer
    outputBuf[0] = FRAME_FLAG_BYTE;
    uint16_t bufferIdx = 1;

    // Push all payload bytes into frame buffer
    for (uint16_t i = 0; i < length; i++) {
        // Check for overflow of frame buffer.
        if (bufferIdx >= UCV_TX_MAX_PAYLOAD_LENGTH) {
            return 0;
        }

        uint8_t data = inputBuf[i];

        // Check in case one of the data bytes happens to randomly be one of the
        // magic bytes. In that event, escape the byte.
        // C.f. https://www.faa.gov/air_traffic/technology/adsb/Archival/media/GDL90_Public_ICD_RevA.PDF,
        // Section 2.2.1.
        if (data == FRAME_FLAG_BYTE || data == CONTROL_ESCAPE_BYTE) {
            // Ensure there is no frame buffer overflow when escaping the bytes
            if (bufferIdx+1 >= UCV_TX_MAX_PAYLOAD_LENGTH) {
                return 0;
            }

            // Place an escape and stuff this byte
            outputBuf[bufferIdx++] = CONTROL_ESCAPE_BYTE;
            outputBuf[bufferIdx++] = data ^ STUFF_BYTE;
        } else {
            outputBuf[bufferIdx++] = data;
        }
    }

    // Add CRC
    outputBuf[bufferIdx++] = frameCrc & 0xFF;
    outputBuf[bufferIdx++] = frameCrc >>8;

    // Add end of frame indication
    outputBuf[bufferIdx++] = FRAME_FLAG_BYTE;

    return bufferIdx;
}


void parseUvcMessage(enum packet_ids packetId, uint8_t buf[UCV_TX_MAX_FRAME_LENGTH])
{
    switch (packetId) {
    case HEARTBEAT:
        ;
        break;
    default:
        // We should never get here. Sound the alarm!

    }
}

enum proto_states parseUvcStream(uint8_t data)
{
    static enum proto_states protoState = START;
    static enum packet_ids packetId;
    enum proto_states protoState_old = protoState;

    static uint16_t payloadLength = 0;
    static uint16_t targetPayloadLength = 0;
    static uint16_t crc = 0;
    static bool isByteEscaped = false;

    static uint8_t buf[UCV_TX_MAX_FRAME_LENGTH];

    // Check if this is the framing byte
    if (data == FRAME_FLAG_BYTE) {
        switch (protoState) {
        case START:
            protoState = UCV_ID;
            payloadLength = 0;
            return protoState;
            break;
        case FINISHED:
            //! Completely valid packet

            // Check CRC
            if (crcCompute(buf, payloadLength) == crc) {
                parseUvcMessage(packetId, buf);
            } else {
            }

            protoState = START;
            return protoState;
            break;
        default:
            // We should never get here. Sound the alarm!
            goto error;
        }
    }

    // Check if this is an escape byte
    if (data == CONTROL_ESCAPE_BYTE) {
        // It is, so simply note that and exit
        isByteEscaped = true;
        return protoState;
    } else if (isByteEscaped == true) {
        // Byte is escaped, so un-escape it
        data ^= STUFF_BYTE;
        isByteEscaped = false;
    }


    switch(protoState) {
    case UCV_ID:
        packetId = data;
        buf[0] = packetId;
        payloadLength = 1;

        switch (packetId) {
        case HEARTBEAT:
            targetPayloadLength = sizeof(HeartbeatV0x00);
            break;
        default:
            // We should never get here. Sound the alarm!
            goto error;
        }

        protoState = UCV_PAYLOAD;
        break;
    case UCV_PAYLOAD:
        buf[payloadLength] = data;

        payloadLength++;

        if (payloadLength == targetPayloadLength) {
            protoState = UCV_CHK1;
        }
        break;
    case UCV_CHK1:
        crc = data & 0xFF;
        protoState = UCV_CHK2;
        break;
    case UCV_CHK2:
        crc |= data<<8;
        protoState = FINISHED;
        break;
    default:
        // We should never get here. Sound the alarm!
        goto error;
    }

    return protoState;

error:

    protoState = START;
    // Alarm state = {proto_state, number of alarms, numer of bytes}
    return protoState_old;

}


#endif // UVC_H
