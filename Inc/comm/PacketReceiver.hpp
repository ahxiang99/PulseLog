#pragma once
#include <cstdint>

#include "SensorPacket.hpp"
#include "crc_calculation.hpp"

enum class State {
    WAIT_HEADER_1,  //  — scanning for 0xAA
    WAIT_HEADER_2,  //  — got 0xAA, waiting for 0x55
    READ_TYPE,
    READ_LENGTH,  //   — got sync, reading length byte
    READ_DATA,    //    — reading N data bytes
    VERIFY_CRC,   //   — got all data, check CRC
    COMPLETE,     //    — valid packet ready to print
};

class PacketReceiverVariant {
   public:
    bool feed(uint8_t byte) {
        if (m_state == State::WAIT_HEADER_1 && byte == 0xAA) {
            m_state    = State::WAIT_HEADER_2;
            m_complete = false;
            return true;
        } else if (m_state == State::WAIT_HEADER_2) {
            if (byte == 0x55) {
                m_state = State::READ_TYPE;  // ✅ got second sync byte
            } else if (byte == 0xAA) {
                m_state = State::WAIT_HEADER_2;  // ✅ new potential header start
            } else {
                m_state = State::WAIT_HEADER_1;  // ✅ not a valid sequence
                m_idx   = 0;
            }
            return true;
        } else if (m_state == State::READ_TYPE) {
            auto type = static_cast<PacketType>(byte);
            if (type == PacketType::VERSION_1 || type == PacketType::VERSION_2) {
                m_type  = static_cast<PacketType>(byte);
                m_state = State::READ_LENGTH;
                return true;
            } else {
                m_type  = PacketType::INVALID;
                m_state = State::WAIT_HEADER_1;
                m_idx   = 0;
                return false;
            }
        } else if (m_state == State::READ_LENGTH) {
            m_length = byte;
            m_state  = State::READ_DATA;
            return true;
        } else if (m_state == State::READ_DATA && m_idx < m_length) {
            m_buf[m_idx++] = byte;
            if (m_idx == m_length) {
                m_state = State::VERIFY_CRC;
            }
            return true;
        } else if (m_state == State::VERIFY_CRC) {
            uint8_t res_crc8 = crc_calculate(m_buf, m_length);
            if (res_crc8 == byte) {
                if (m_type == PacketType::VERSION_1) {
                    SensorPacketV1 p;
                    memcpy(&p, m_buf, m_length);
                    m_packet = p;
                } else if (m_type == PacketType::VERSION_2) {
                    SensorPacketV2 p;
                    memcpy(&p, m_buf, m_length);
                    m_packet = p;
                } else {
                    m_state = State::WAIT_HEADER_1;
                    m_idx   = 0;
                    return false;
                }
                m_state    = State::WAIT_HEADER_1;
                m_idx      = 0;
                m_complete = true;
                return true;  // ← packet complete
            } else {
                m_state    = State::WAIT_HEADER_1;
                m_idx      = 0;
                m_complete = false;
                return false;  // ← CRC mismatch
            }
        } else {
            m_state    = State::WAIT_HEADER_1;
            m_idx      = 0;
            m_complete = false;
            return false;  // ← CRC mismatch
        }
    }

    const std::variant<SensorPacketV1, SensorPacketV2>& packet() const {
        return m_packet;
    }

    bool isComplete() const {
        return m_complete;
    }

   private:
    State                                        m_state{State::WAIT_HEADER_1};
    PacketType                                   m_type{PacketType::INVALID};
    uint8_t                                      m_length{0};
    uint8_t                                      m_idx{0};
    uint8_t                                      m_buf[16]{};
    std::variant<SensorPacketV1, SensorPacketV2> m_packet{};
    bool                                         m_complete{false};
};

template <typename T>
class PacketReceiver {
   public:
    bool feed(uint8_t byte) {
        if (m_state == State::WAIT_HEADER_1 && byte == 0xAA) {
            m_state    = State::WAIT_HEADER_2;
            m_complete = false;
            return true;
        } else if (m_state == State::WAIT_HEADER_2) {
            if (byte == 0x55) {
                m_state = State::READ_LENGTH;  // ✅ got second sync byte
            } else if (byte == 0xAA) {
                m_state = State::WAIT_HEADER_2;  // ✅ new potential header start
            } else {
                m_state = State::WAIT_HEADER_1;  // ✅ not a valid sequence
                m_idx   = 0;
            }
            return true;
        } else if (m_state == State::READ_LENGTH) {
            m_length = byte;
            m_state  = State::READ_DATA;
            return true;
        } else if (m_state == State::READ_DATA && m_idx < m_length) {
            m_buf[m_idx++] = byte;
            if (m_idx == m_length) {
                m_state = State::VERIFY_CRC;
            }
            return true;
        } else if (m_state == State::VERIFY_CRC) {
            uint8_t res_crc8 = crc_calculate(m_buf, m_length);
            if (res_crc8 == byte) {
                std::memcpy(&m_packet, m_buf, sizeof(T));  // ← here
                m_state    = State::WAIT_HEADER_1;
                m_idx      = 0;
                m_complete = true;
                return true;  // ← packet complete
            } else {
                m_state    = State::WAIT_HEADER_1;
                m_idx      = 0;
                m_complete = false;
                return false;  // ← CRC mismatch
            }
        } else {
            m_state    = State::WAIT_HEADER_1;
            m_idx      = 0;
            m_complete = false;
            return false;  // ← CRC mismatch
        }
    }

    const T& packet() const {
        return m_packet;
    }

    bool isComplete() const {
        return m_complete;
    }

   private:
    State   m_state{State::WAIT_HEADER_1};
    uint8_t m_length{0};
    uint8_t m_idx{0};
    uint8_t m_buf[sizeof(T)]{};
    T       m_packet{};
    bool    m_complete{false};
};
