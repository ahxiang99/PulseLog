#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "logger.hpp"

enum class AppMode : uint8_t {
	SendPacket = 0,
	Console = 1,
	CliMode = 2,
	COUNT = 3
};

struct AppMode_Table {
	AppMode mode;
	const char *name;
};

static constexpr AppMode_Table namingTable[static_cast<uint8_t>(AppMode::COUNT)] = {
	{.mode = AppMode::SendPacket, .name = "Packet Mode"}, {AppMode::Console, "Console Mode"}, {AppMode::CliMode, "CLI Mode"}};

static inline void onButtonPress(void *ctx)
{
	AppMode curMode = static_cast<std::atomic<AppMode> *>(ctx)->load(std::memory_order_relaxed);
	if (curMode == AppMode::CliMode) {
		curMode = AppMode::SendPacket;
	} else {
		uint8_t i = static_cast<uint8_t>(curMode) + 1;
		curMode = static_cast<AppMode>(i);
	}
	LOG_INFO("Changing Mode: {}", namingTable[static_cast<uint8_t>(curMode)].name);
	static_cast<std::atomic<AppMode> *>(ctx)->store(curMode, std::memory_order_relaxed);
}
