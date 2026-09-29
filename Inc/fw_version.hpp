#pragma once

#include <cstdint>

namespace fw
{

inline constexpr uint32_t kVersionMagic = 0x504C4647; // "PLFG" (PulseLog FirmwareGuard)

inline constexpr uint8_t kFwVersionMajor = 0;
inline constexpr uint8_t kFwVersionMinor = 1;
inline constexpr uint8_t kFwVersionPatch = 0;
inline constexpr uint32_t kFwBuildNumber = 1;

struct VersionInfo {
	uint32_t magic;
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	uint8_t reserved;
	uint32_t build_number;
};

/* Placed in the .fw_version linker section (stm32f401xe_flash.ld), which is
   positioned immediately after the fixed-size .isr_vector, so this struct
   lands at a deterministic flash offset that external tooling can read
   without parsing the ELF (e.g. `objcopy -O binary` + fixed-offset read). */
inline constexpr VersionInfo g_fw_version __attribute__((section(".fw_version"), used)){
	.magic = kVersionMagic,
	.major = kFwVersionMajor,
	.minor = kFwVersionMinor,
	.patch = kFwVersionPatch,
	.reserved = 0,
	.build_number = kFwBuildNumber,
};

} // namespace fw
