#pragma once

#include <stdint.h>
#include <AP_Common/AP_Common.h>

/*
  Wire format definitions for the syslink protocol spoken between the STM32 and the nRF51822 radio co-processor on Crazyflie 2.x. See AP_Syslink.md.

  All multi-byte fields are little-endian.
 */
namespace AP_Syslink_Protocol {

// Framing
static constexpr uint8_t SYNC0 = 0xBC;
static constexpr uint8_t SYNC1 = 0xCF;

// LEN is a single byte, so DATA can be at most 255 bytes on the wire.
static constexpr uint16_t MAX_DATA_LEN = 255;

// Bytes of framing overhead around DATA: SYNC0, SYNC1, TYPE, LEN + 2 checksum.
static constexpr uint8_t FRAME_OVERHEAD = 6;

/*
  Largest MAVLink chunk that fits one radio packet: the 252 byte ESB payload less the one byte on-air marker that separates MAVLink traffic from the
  CRTP control packets the nRF51 answers locally. A longer chunk is dropped by the nRF51 rather than truncated.
 */
static constexpr uint8_t MAVLINK_CHUNK_MAX = 251;

// Usable depth of the nRF51's unicast transmit queue, as reported by RADIO_MAVLINK_SPACE. P2P Broadcasts are not queued and are not counted.
static constexpr uint8_t MAVLINK_TX_SLOTS = 5;

/*
  Packet types. The high nibble is the group (0x00 radio, 0x10 power management, 0x20 one-wire, 0x30 system, 0xF0 debug).
 */
enum class Type : uint8_t {
    RADIO_RAW               = 0x00,
    RADIO_CHANNEL           = 0x01,
    RADIO_DATARATE          = 0x02,
    RADIO_CONTWAVE          = 0x03,
    RADIO_RSSI              = 0x04,
    RADIO_ADDRESS           = 0x05,
    RADIO_RAW_BROADCAST     = 0x06,
    RADIO_POWER             = 0x07,
    RADIO_P2P               = 0x08,
    RADIO_P2P_ACK           = 0x09,
    RADIO_P2P_BROADCAST     = 0x0A,
    RADIO_READY             = 0x0B,
    RADIO_MAVLINK           = 0x0C,
    RADIO_MAVLINK_BROADCAST = 0x0D,
    RADIO_MAVLINK_SPACE     = 0x0E,

    PM_SOURCE               = 0x10,
    PM_ONOFF_SWITCHOFF      = 0x11,
    PM_BATTERY_VOLTAGE      = 0x12,
    PM_BATTERY_STATE        = 0x13,
    PM_BATTERY_AUTOUPDATE   = 0x14,
    PM_SHUTDOWN_REQUEST     = 0x15,
    PM_SHUTDOWN_ACK         = 0x16,
    PM_LED_ON               = 0x17,
    PM_LED_OFF              = 0x18,
    PM_DECKCTRL_DFU         = 0x19,

    OW_SCAN                 = 0x20,
    OW_GETINFO              = 0x21,
    OW_READ                 = 0x22,
    OW_WRITE                = 0x23,

    SYS_NRF_VERSION         = 0x30,

    DEBUG_PROBE             = 0xF0,
};

// RADIO_DATARATE values
enum class DataRate : uint8_t {
    RATE_250K = 0,
    RATE_1M   = 1,
    RATE_2M   = 2,
};

// The radio address is 5 bytes, sent little-endian.
static constexpr uint8_t ADDRESS_LEN = 5;

/*
  PM_BATTERY_STATE payload. TEMP is only present when the nRF51 is built with PM_SYSLINK_INCLUDE_TEMP, so the packet is either 13 or 17 bytes

  ISET is *charge* current, not discharge. TEMP is the nRF51 die temperature, not the battery's - see AP_Syslink.md.
 */
struct PACKED BatteryState {
    uint8_t flags;      // bit0 charging, bit1 USB powered, bit2 can charge
    float   vbat;       // volts
    float   iset;       // charge current, milliamperes
    float   temp;       // nRF51 die temperature, degrees C (optional)
};

static constexpr uint8_t BATTERY_STATE_LEN          = 13;
static constexpr uint8_t BATTERY_STATE_LEN_WITH_TEMP = 17;

// PM_BATTERY_STATE flag bits
static constexpr uint8_t BATTERY_FLAG_CHARGING    = (1U << 0);
static constexpr uint8_t BATTERY_FLAG_USB_POWERED = (1U << 1);
static constexpr uint8_t BATTERY_FLAG_CAN_CHARGE  = (1U << 2);

// DEBUG_PROBE response payload
struct PACKED DebugProbeData {
    uint8_t addr_set;   // 1 if a RADIO_ADDRESS command has been received
    uint8_t chan_set;   // 1 if a RADIO_CHANNEL command has been received
    uint8_t rate_set;   // 1 if a RADIO_DATARATE command has been received
    uint8_t dropped;    // 1 if UART data has been dropped
    uint8_t uart_err;   // UART error flags
    uint8_t uart_cnt;   // UART error count
    uint8_t cksum1;     // syslink RX checksum 1 error count
    uint8_t cksum2;     // syslink RX checksum 2 error count
};

static constexpr uint8_t DEBUG_PROBE_LEN = 8;

} // namespace AP_Syslink_Protocol
