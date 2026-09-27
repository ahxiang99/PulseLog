#pragma once
#include "SHT40X.hpp"
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>

#include "crc_calculation.hpp"

enum class PacketType {
	PKT_SHT40 = 0xB0U,
	PKT_STTS2H = 0xB1U,
	PKT_ENV_SENSOR_DATA = 0xC0U,
	INVALID = 0xFFU
};

template <typename T, PacketType packType> class Packet
{
	std::array<uint8_t, sizeof(T) + 5> m_bytes;

      public:
	explicit Packet(const T &data)
	{
		m_bytes[0] = 0xAAU;
		m_bytes[1] = 0x55U;
		m_bytes[2] = static_cast<uint8_t>(packType);
		m_bytes[3] = sizeof(T);
		std::memcpy(&m_bytes[4], &data, sizeof(T));
		m_bytes[4 + sizeof(T)] = crc_calculate(&m_bytes[4], sizeof(T));
	}

	[[nodiscard]] const uint8_t *raw() const
	{
		return m_bytes.data();
	}

	[[nodiscard]] size_t size() const
	{
		return m_bytes.size();
	}
};

struct __attribute__((packed)) Env_Sensor_Data {
	uint16_t last_rx_tick;
	uint16_t seq;
	int16_t temp_x100;
	int16_t rh_x100;
};

struct SensorPacketV1 {
	SHT40X::SensorData m_data;

	static std::string header()
	{
		return "temperature,humidity\n";
	}
};

struct SensorPacketV2 {
	float_t temperature;

	static std::string header()
	{
		return "temperature\n";
	}

	[[nodiscard]] std::string toCsv() const
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(2) << temperature << ',' << '\n';
		return oss.str();
	}
};
