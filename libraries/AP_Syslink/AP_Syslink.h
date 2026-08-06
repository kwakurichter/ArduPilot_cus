#pragma once

#include "AP_Syslink_config.h"

#if AP_SYSLINK_ENABLED

#include "AP_Syslink_Protocol.h"
#include "AP_Syslink_MAVLinkPort.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

/*
  Driver for the nRF51822 radio co-processor on Crazyflie 2.x, reached over a
  UART running the syslink framing protocol.

  See AP_Syslink.md.
 */
class AP_Syslink
{
public:
    AP_Syslink();

    CLASS_NO_COPY(AP_Syslink);

    static AP_Syslink *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    bool enabled() const { return _enable != 0; }

    // True once the driver thread is running.
    bool initialised() const { return _initialised; }

    // True once the driver thread has opened the port.
    bool port_ready() const { return _port_ready; }

    /*
      Queue one syslink packet; framing and checksum are added here. Safe to call from any thread.

      Returns false if the packet does not fit the transmit buffer, in which  case it is dropped whole rather than truncated 
      (a partial frame would desynchronise the nRF51's parser).
     */
    bool send_packet(AP_Syslink_Protocol::Type type, const uint8_t *data, uint8_t len);
    bool send_packet(AP_Syslink_Protocol::Type type) { return send_packet(type, nullptr, 0); }

    // Handler for one received packet, called from the syslink thread. Takes the packet type so one handler can serve several types.
    FUNCTOR_TYPEDEF(PacketHandler, void, uint8_t, const uint8_t *, uint8_t);

    /*
      Register a handler for one packet type. Returns false if the table is full or that type already has a handler. Handlers run on the syslink
      thread, so they must not block.
     */
    bool register_handler(AP_Syslink_Protocol::Type type, PacketHandler handler);

    // Ask the nRF51 for a DEBUG_PROBE response.
    bool request_debug_probe() { return send_packet(AP_Syslink_Protocol::Type::DEBUG_PROBE); }

    struct Stats {
        uint32_t bcast_tx;           // peer broadcasts queued
        uint32_t bcast_rx;           // peer broadcasts received
        uint32_t bcast_rejected;     // broadcasts refused, empty or over length
        uint32_t rx_bytes;           // raw bytes read from the UART
        uint32_t tx_bytes;           // raw bytes written to the UART
        uint32_t rx_packets;         // well-formed packets received
        uint32_t rx_cksum_errors;    // Fletcher-8 mismatches
        uint32_t rx_unhandled;       // received with no registered handler
        uint32_t tx_packets;         // packets queued
        uint32_t tx_dropped;         // packets dropped, transmit buffer full
        uint32_t flowctrl_timeouts;  // sent anyway after RTS stayed deasserted
    };
    const Stats &get_stats() const { return _stats; }

    // True once the nRF51 has echoed RADIO_READY, proving the link works.
    bool link_up() const { return _config_state > ConfigState::READY; }

    // True once the whole boot sequence has been sent.
    bool configured() const { return _config_state == ConfigState::DONE; }

#if AP_SERIALMANAGER_REGISTER_ENABLED
    // The virtual serial port carrying MAVLink over the radio.
    AP_Syslink_MAVLinkPort &get_mavlink_port() { return _mavlink_port; }
#endif

    /*
      Low byte of the radio address (SYSL_ADDR). Doubles as this vehicle's
      peer identity: it is what distinguishes one Crazyflie from another on a
      shared channel, so anything needing a node id should use it rather than
      keep a second parameter that can disagree with the radio.
     */
    uint8_t get_address() const { return uint8_t(constrain_int16(_address.get(), 0, 255)); }

    // Bytes per second the GCS should assume for this link (SYSL_BW).
    uint16_t link_bw() const { return uint16_t(_link_bw.get()); }

    // Whether to pack several whole MAVLink frames into one radio chunk.
    bool pack_frames() const { return option_set(Option::PACK_FRAMES); }

    /*
      Whether the virtual port claims flow control to the GCS. Off by default:
      it makes AP_Logger send 10 LOG_DATA per call instead of 1, which produces
      far more than this radio can carry and collapses the link.
     */
    bool report_flow_control() const { return option_set(Option::REPORT_FLOW_CTRL); }

    /*
      Peer-to-peer broadcast, for AP_SwarmMesh and anything else that needs to
      reach every peer on the shared address.

      This is transport only. send_broadcast() adds the syslink header and
      Fletcher-8 checksum and nothing else; inbound packets reach the handler
      with both already stripped and verified. The payload is never inspected
      in either direction, and the nRF51 forwards it verbatim, so what one
      vehicle sends is byte for byte what its peers receive.

      Broadcasts are transmitted by the nRF51 immediately rather than queued.
      They consume no unicast transmit slot and so cannot fail for lack of
      radio room, and they do not compete with telemetry for the 5 deep queue.
      They are also unacked and never retried: any reliability, ordering or
      deduplication is the caller's to build.

      Use get_address() for this vehicle's node id - it is the radio address
      low byte, so it cannot disagree with what peers actually see.
     */
    static constexpr uint8_t broadcast_max_len() { return AP_Syslink_Protocol::MAVLINK_CHUNK_MAX; }

    /*
      Queue one broadcast. Returns false if the packet is empty, longer than
      broadcast_max_len(), the radio is not configured yet, or the transmit
      buffer is full - in which case it is dropped whole rather than
      truncated, since a partial frame would desynchronise the nRF51.
     */
    bool send_broadcast(const uint8_t *data, uint8_t len);

    // Sink for inbound broadcasts, called on the syslink thread with the
    // payload only. Must not block. Replaces any previous handler.
    FUNCTOR_TYPEDEF(BroadcastHandler, void, const uint8_t *, uint8_t);
    void set_broadcast_handler(BroadcastHandler handler);

    /*
      Power state from the nRF51's last PM_BATTERY_STATE report. Valid only
      once battery_time_ms() is non-zero.
     */
    bool is_charging() const;
    bool is_usb_powered() const;
    uint32_t battery_time_ms() const { return _batt_time_ms; }

    // Most recent DEBUG_PROBE response; probe_time_ms is 0 if none received.
    const AP_Syslink_Protocol::DebugProbeData &get_debug_probe() const { return _probe; }
    uint32_t get_debug_probe_time_ms() const { return _probe_time_ms; }

private:
    static AP_Syslink *_singleton;

    // parameters
    AP_Int8 _enable;
    AP_Int8 _options;

    enum class Option : uint8_t {
        USE_FLOW_CONTROL = (1U << 0),
        LOG_STATS        = (1U << 1),
        PACK_FRAMES      = (1U << 2),
        REPORT_FLOW_CTRL = (1U << 3),
    };
    bool option_set(Option opt) const { return (uint8_t(_options.get()) & uint8_t(opt)) != 0; }

    bool init_port();
    void thread_main();
    void receive_bytes();
    void send_pending();
    void parse_byte(uint8_t b);
    void dispatch(uint8_t type, const uint8_t *data, uint8_t len);
    bool flow_control_ok();
    void update_stats_1hz();
    void handle_debug_probe(uint8_t type, const uint8_t *data, uint8_t len);
    void handle_config_echo(uint8_t type, const uint8_t *data, uint8_t len);
    void handle_battery_state(uint8_t type, const uint8_t *data, uint8_t len);
    void handle_broadcast(uint8_t type, const uint8_t *data, uint8_t len);

    void update_config();
    void send_config_step();
    void advance_config();

    static void fletcher8(const uint8_t *data, uint16_t len, uint8_t &c0, uint8_t &c1);

    AP_HAL::UARTDriver *_uart;
    bool _initialised;
    bool _port_ready;

    // receive state machine
    enum class RxState : uint8_t {
        SYNC0,
        SYNC1,
        TYPE,
        LEN,
        DATA,
        CKSUM0,
        CKSUM1,
    };
    RxState _rx_state;
    uint8_t _rx_type;
    uint8_t _rx_len;
    uint16_t _rx_idx;
    uint8_t _rx_ck0;        // running checksum
    uint8_t _rx_ck1;
    uint8_t _rx_ck0_recv;   // checksum as received
    uint8_t _rx_data[AP_Syslink_Protocol::MAX_DATA_LEN];

    // outgoing framed bytes, drained to the UART by the driver thread
    ByteBuffer *_tx_buf;
    HAL_Semaphore _tx_sem;

    static const uint8_t MAX_HANDLERS = 16;
    struct HandlerEntry {
        PacketHandler handler;
        uint8_t type;
        bool used;
    } _handlers[MAX_HANDLERS];
    HAL_Semaphore _handler_sem;

#if AP_SERIALMANAGER_REGISTER_ENABLED
    AP_Syslink_MAVLinkPort _mavlink_port;
#endif

    BroadcastHandler _broadcast_handler;
    bool _have_broadcast_handler;

    Stats _stats;
    AP_Syslink_Protocol::DebugProbeData _probe;
    uint32_t _probe_time_ms;

    // power management state from PM_BATTERY_STATE
    uint32_t _last_battery_ms;
    uint32_t _batt_time_ms;
    uint8_t _batt_flags;
    bool _batt_warned;

    uint32_t _flowctrl_blocked_ms;
    bool _flowctrl_failed;      // line stuck deasserted; gate disabled
    uint32_t _last_1hz_ms;

    /*
      Boot sequence. The nRF51 sends nothing at all until it has received one valid syslink packet, so this must complete before anything else works.
     */
    enum class ConfigState : uint8_t {
        READY,          // RADIO_READY, lifts the transmit gate; echoed
        AUTOUPDATE,     // PM_BATTERY_AUTOUPDATE, enables battery and RSSI
        CHANNEL,
        DATARATE,
        ADDRESS,
        POWER,
        DONE,
    };
    ConfigState _config_state = ConfigState::READY;
    bool _config_sent;
    uint32_t _config_sent_ms;
    uint8_t _config_retries;
    // 0xFF is not a valid packet type, so it can never match a real echo
    uint8_t _config_expect = 0xFF;
    uint8_t _config_echo_type = 0xFF;
    uint32_t _config_echo_ms;

    // parameters
    AP_Int16 _channel;
    AP_Int8 _datarate;
    AP_Int16 _address;
    AP_Int8 _txpower;
    AP_Int16 _link_bw;
    AP_Int8 _batt_instance;
};

namespace AP {
    AP_Syslink *syslink();
};

#endif // AP_SYSLINK_ENABLED
