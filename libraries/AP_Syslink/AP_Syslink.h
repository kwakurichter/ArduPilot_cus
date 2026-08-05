#pragma once

#include "AP_Syslink_config.h"

#if AP_SYSLINK_ENABLED

#include "AP_Syslink_Protocol.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Param/AP_Param.h>

/*
  Driver for the nRF51822 radio co-processor on Crazyflie 2.x, reached over a
  UART running the syslink framing protocol.

  This class owns the UART outright: it frames outgoing packets, deframes
  incoming ones and dispatches them by type to registered handlers. Nothing
  else may open that port.

  init() must run before GCS::setup_uarts(), because later phases register a
  virtual MAVLink port that the GCS binds to during that call.

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

    // True once the UART is open and the driver thread is running.
    bool initialised() const { return _initialised; }

    /*
      Queue one syslink packet; framing and checksum are added here. Safe to
      call from any thread.

      Returns false if the packet does not fit the transmit buffer, in which
      case it is dropped whole rather than truncated - a partial frame would
      desynchronise the nRF51's parser.
     */
    bool send_packet(AP_Syslink_Protocol::Type type, const uint8_t *data, uint8_t len);
    bool send_packet(AP_Syslink_Protocol::Type type) { return send_packet(type, nullptr, 0); }

    // Handler for one received packet, called from the syslink thread.
    FUNCTOR_TYPEDEF(PacketHandler, void, const uint8_t *, uint8_t);

    /*
      Register a handler for one packet type. Returns false if the table is
      full or that type already has a handler. Handlers run on the syslink
      thread, so they must not block.
     */
    bool register_handler(AP_Syslink_Protocol::Type type, PacketHandler handler);

    // Ask the nRF51 for a DEBUG_PROBE response.
    bool request_debug_probe() { return send_packet(AP_Syslink_Protocol::Type::DEBUG_PROBE); }

    struct Stats {
        uint32_t rx_packets;         // well-formed packets received
        uint32_t rx_cksum_errors;    // Fletcher-8 mismatches
        uint32_t rx_unhandled;       // received with no registered handler
        uint32_t tx_packets;         // packets queued
        uint32_t tx_dropped;         // packets dropped, transmit buffer full
        uint32_t flowctrl_timeouts;  // sent anyway after RTS stayed deasserted
    };
    const Stats &get_stats() const { return _stats; }

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
    };
    bool option_set(Option opt) const { return (uint8_t(_options.get()) & uint8_t(opt)) != 0; }

    void thread_main();
    void receive_bytes();
    void send_pending();
    void parse_byte(uint8_t b);
    void dispatch(uint8_t type, const uint8_t *data, uint8_t len);
    bool flow_control_ok();
    void update_stats_1hz();
    void handle_debug_probe(const uint8_t *data, uint8_t len);

    static void fletcher8(const uint8_t *data, uint16_t len, uint8_t &c0, uint8_t &c1);

    AP_HAL::UARTDriver *_uart;
    bool _initialised;

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

    static const uint8_t MAX_HANDLERS = 8;
    struct HandlerEntry {
        PacketHandler handler;
        uint8_t type;
        bool used;
    } _handlers[MAX_HANDLERS];
    HAL_Semaphore _handler_sem;

    Stats _stats;
    AP_Syslink_Protocol::DebugProbeData _probe;
    uint32_t _probe_time_ms;

    uint32_t _flowctrl_blocked_ms;
    bool _flowctrl_failed;      // line stuck deasserted; gate disabled
    uint32_t _last_1hz_ms;
};

namespace AP {
    AP_Syslink *syslink();
};

#endif // AP_SYSLINK_ENABLED
