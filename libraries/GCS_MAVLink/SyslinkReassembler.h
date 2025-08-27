// SyslinkReassembler.h
#ifndef SYSLINK_REASSEMBLER_H
#define SYSLINK_REASSEMBLER_H

#include <vector>
#include <map>
#include <functional> // For std::function
#include <AP_HAL/AP_HAL.h> // For AP_HAL::millis()

typedef std::vector<uint8_t> uint8_t_vector;

class SyslinkToMAVLinkReassembler {
public:
    static constexpr uint8_t SYSLINK_SYNC1 = 0xBC;
    static constexpr uint8_t SYSLINK_SYNC2 = 0xCF;
    static constexpr uint8_t EXPECTED_SYSLINK_TYPE_RADIO = 0x00;
    static constexpr uint8_t EXPECTED_SYSLINK_TYPE_MAVLINK = 0x0B;
    static constexpr uint8_t EXPECTED_SYSLINK_TYPE_P2P = 0x08;
    static constexpr uint8_t EXPECTED_SYSLINK_TYPE_P2P_BROADCAST = 0x0A;   

    enum class ParseState {
        WAIT_SYNC1,
        WAIT_SYNC2,
        READ_TYPE_LENGTH,
        WAIT_CRTP_HEADER,
        WAIT_P2P_CRTP_HEADER,
        READ_PAYLOAD_AND_CRC
    };

    SyslinkToMAVLinkReassembler() : state(ParseState::WAIT_SYNC1), syslink_payload_bytes_expected(0) {}

    // Processes an incoming byte.
    // Returns true if the byte 'c' was consumed by the Syslink state machine.
    // mavlink_byte_pusher: A callback to push reassembled MAVLink bytes for further parsing.
    bool process_byte(uint8_t c, 
                    std::function<void(uint8_t mav_byte)> mavlink_byte_pusher,
                    std::function<void(const uint8_t* p2p_payload, uint8_t len)> p2p_packet_handler);

private:
    ParseState state;
    uint8_t_vector current_syslink_frame_buffer; // Stores from SYNC1 up to CRC
    uint8_t syslink_type_byte;
    uint8_t syslink_length_field; // Length of (fragment_header + MAVLink_data_slice)
    uint16_t syslink_payload_bytes_expected; // Expected bytes for (fragment_header + MAVLink_data_slice + CRC)

    struct FragmentBuffer {
        uint16_t original_len;
        uint8_t total_frags;
        std::map<uint8_t, uint8_t_vector> frags; // seq -> payload_slice
        uint32_t last_active_ms;
    };
    std::map<uint16_t, FragmentBuffer> reassembly_buffers; // full_id -> FragmentBuffer

    bool check_fletcher8(const uint8_t* data, size_t len_for_check, uint8_t crc0_expected, uint8_t crc1_expected);
    void handle_complete_syslink_fragment(const uint8_t* frag_payload_data, // Points to start of 6-byte frag_header
                                          uint8_t frag_payload_len,      // Length of (frag_header + MAVLink_slice)
                                          std::function<void(uint8_t mav_byte)> mavlink_byte_pusher);
    void reset_parser_state();
    void cleanup_stale_reassembly_buffers(); // Periodically call this
};

#endif // SYSLINK_REASSEMBLER_H