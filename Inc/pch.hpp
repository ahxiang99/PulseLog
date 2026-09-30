#pragma once

/* C++ Standard Libraries (heavy to parse in embedded) */
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <string_view>
#include <span>

/* Define ConstantExpr */
const size_t CHUNK_SIZE = 64;
const size_t BUFF_SIZE = 128;
const uint32_t HSI_Freq_Hz = 16000000;

constexpr bool kSensorEnable = true;
constexpr bool kSendPacket = false;

// Define State
#define CONSOLE_APP
// #define MAX30102_OXI_EN
// #define WATCH_DOG_ENABLE

/* Custom C++ Library */
#include "FloatIntExtraction.hpp"
#include "RegisterUtils.hpp"
#include "Result.hpp"
#include "drivers.hpp"
#include "logger.hpp"

/* Custom C Library */
#include "bit_utils.h"
#include "cmsis.h"
#include "low-level/nvic.h"
#include "low-level/pwr_bitfields.h"
#include "low-level/pwr_registers.h"
#include "low-level/rtc_registers.h"
