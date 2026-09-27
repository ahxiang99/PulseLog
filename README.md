# PulseLog

A bare-metal C++20 firmware project for the STM32F401XE (ARM Cortex-M4) demonstrating a multi-layer driver architecture with DMA-enabled I2C and UART, dual temperature sensors, and zero HAL dependency.

## What This Project Does

The firmware reads temperature/humidity from two I2C sensors (SHT40-AD1B and STTS2H) on a 250 ms interval, then transmits structured binary packets to a host PC over UART2 using DMA. All peripheral access is via hand-written register drivers — no ST HAL or LL libraries.

```
SHT40-AD1B ──┐
              ├── I2C1 (DMA) ──── STM32F401XE ──── UART2 (DMA) ──── PC
STTS2H     ──┘                        │
                                   TIM3 (250 ms tick)
```

## Key Features

- **Zero HAL** — direct CMSIS register access via `volatile` struct pointers
- **DMA everywhere** — I2C receive and UART transmit both use DMA, freeing the CPU
- **Layered drivers** — thin C low-level header layer consumed by a C++ class layer
- **Zero-cost abstractions** — CRTP, `constexpr`, and type-erasure instead of virtual dispatch
- **No dynamic allocation** — `std::array`, stack, and static storage only
- **Structured packet protocol** — versioned binary packets (`PacketV2<T, VERSION>`) for PC-side parsing
- **Optional CLI** — UART command-line interface, toggled at compile time

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32F401XE, ARM Cortex-M4 @ 84 MHz |
| Flash | 512 KB |
| RAM | 96 KB |
| FPU | Hardware FPV4-SP-D16 |
| Sensor 1 | SHT40-AD1B — temperature + humidity (I2C, addr 0x44) |
| Sensor 2 | STTS2H — temperature (I2C, addr 0x3F) |
| UART | USART2, PA2 (TX) / PA3 (RX), 115200 baud |
| I2C | I2C1, PB8 (SCL) / PB9 (SDA), 100 kHz |
| LED | PA5 — toggles each timer tick as a heartbeat |
| Debugger | ST-Link V2/V2-1 |

## Project Structure

```
PulseLog/
├── Src/
│   └── main.cpp              # Application entry point, ISR routing
├── Inc/
│   ├── Sht40ad1b.hpp         # SHT40 sensor driver (state machine)
│   ├── stts2h.hpp            # STTS2H sensor driver
│   ├── SensorPacket.hpp      # Binary packet protocol (PacketV2<T>)
│   ├── drivers.hpp           # Global Drivers struct (singleton accessor)
│   └── pch.hpp               # Precompiled header
├── Middleware/
│   ├── logger.hpp            # Singleton logger ({} placeholders, no printf)
│   ├── cli.hpp               # UART command-line interface
│   └── RingBuffer.hpp        # Lock-free SPSC ring buffer template
├── drivers/
│   ├── common/               # RegisterUtils, FloatIntExtraction, bit_utils
│   ├── gpio/                 # Stm32GpioPin (CRTP-based)
│   ├── uart/                 # Stm32Uart → InterruptUart / DmaUart
│   ├── i2c/                  # Stm32I2C → InterruptI2C / DmaI2C
│   ├── timer/                # Stm32Timer (TIM3)
│   ├── DMA/                  # IDma / Dma
│   ├── systick/              # MySysTick (1 ms resolution)
│   ├── rcc/                  # RCC low-level C helpers
│   └── nvic/                 # NVIC helpers
├── CMakeLists.txt
├── CMakePresets.json
└── stm32f401xe_flash.ld      # Linker script
```

## Driver Architecture

Each peripheral follows a two-layer pattern:

**Low-level C layer** (`drivers/<peripheral>/low-level/`):
- `*_registers.h` — memory-mapped register struct + base address macros
- `*_bitfields.h` — bit-mask and shift constants
- `*_types.h` — peripheral-specific enums and config structs

**C++ driver layer** (`drivers/<peripheral>/cpp/`):

| Peripheral | Interface | Concrete classes |
|---|---|---|
| GPIO | `IGpio` | `Stm32GpioPin` |
| UART | `IUart` → `UartBase` → `Stm32Uart` | `InterruptUart`, `DmaUart` |
| I2C | `II2C` → `Stm32I2C` | `InterruptI2C`, `DmaI2C` |
| Timer | `ITimer` | `Stm32Timer` |
| DMA | `IDma` | `Dma` |
| SysTick | — | `MySysTick` |

**Init pattern** — two-phase to avoid static-init ordering issues:
```cpp
// Phase 1: declare globals (zero-initialized)
DmaI2C i2c1;

// Phase 2: configure and initialize in Init_Driver()
i2c1.setVariable(I2C_1, i2c_cfg);
i2c1.initialize();    // post-init hook wires up DMA channels automatically
```

**ISR routing** — all ISR functions live in `Src/main.cpp` and delegate directly:
```cpp
extern "C" void DMA1_Stream5_IRQHandler() {
    getDrivers().i2c1.handleRxDmaInterrupt();
}
```

## Getting Started

### Prerequisites

- [ARM GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (`arm-none-eabi-gcc` ≥ 12)
- CMake ≥ 3.20
- ST-Link V2/V2-1 debugger + [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) or OpenOCD

Optional (for VS Code debugging):
- [STM32CubeIDE VS Code Extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension)
- A debug configuration is pre-configured in `.vscode/launch.json`

### Build

```bash
git clone https://github.com/ahxiang99/PulseLog.git
cd PulseLog

# Configure (uses CMakePresets.json "default" preset)
cmake --preset default

# Build
cmake --build build
```

Post-build steps print memory usage and generate:
- `build/baremetal-hardware-demo.elf`
- `build/baremetal-hardware-demo.hex`
- `build/baremetal-hardware-demo.bin`

### Flash

**With STM32CubeProgrammer:**
```bash
STM32_Programmer_CLI -c port=SWD -d build/baremetal-hardware-demo.bin 0x08000000 -v -rst
```

**With OpenOCD:**
```bash
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg \
  -c "program build/baremetal-hardware-demo.bin 0x08000000 verify reset exit"
```

**With VS Code:** press **F5** — the launch configuration builds and flashes via ST-Link GDB server.

### Verify

Connect a serial terminal to USART2 (115200 8N1). With `kSendPacket = false`, readable log output appears:

```
[INFO] SHT40: Temp:25, Rh:48
[INFO] STTS2H: Temp:25
```

With `kSendPacket = true` (default), the firmware emits raw binary `PacketV2` frames instead, suited for a PC-side parser.

## Compile-Time Feature Flags

All flags are `constexpr bool` at the top of `Src/main.cpp`:

| Flag | Default | Effect |
|------|---------|--------|
| `kSensorEnable` | `true` | Enable SHT40 + STTS2H sensor reads |
| `kSendPacket` | `true` | Send binary `PacketV2` frames; `false` → human-readable log |
| `kCliEnable` | `false` | Enable UART CLI (`get-temp`, `help` commands) |

## Sensor Application

Both sensors share I2C1. The main loop uses a command queue (`SpscRingBuffer<I2CCommand, 4>`) so only one I2C transaction is in flight at a time, guarded by `std::atomic_bool g_cmd_complete`.

**SHT40-AD1B state machine** (`Inc/Sht40ad1b.hpp`):
```
IDLE ──read()──► MEASURING ──30 ms──► WAIT_DATA ──DMA done──► DATA_READY ──► IDLE
```

Each state transition is driven by `ProcessData()` called from the main loop. CRC-8 validates the 6-byte response before accepting temperature and humidity values.

## Middleware

### Logger

```cpp
Logger::Init(uart_ref);
LOG_INFO("Temp:{}, Rh:{}", sensor.getValue().temperature, sensor.getValue().humidity);
```

Uses `{}` placeholders. Floats are printed via `FloatIntExtraction` (splits into integer + decimal parts) to avoid newlib `%f` overhead.

### Ring Buffer

```cpp
SpscRingBuffer<I2CCommand, 4> cmd_queue;   // power-of-2 capacity required
cmd_queue.push({fn_ptr, &ctx});
I2CCommand cmd;
if (cmd_queue.pop(cmd)) cmd.fn(cmd.ctx);
```

Lock-free single-producer / single-consumer; safe between main loop and ISR context.

## Peripheral Configuration

| Peripheral | Pins | Speed | Notes |
|---|---|---|---|
| USART2 | PA2 (TX), PA3 (RX) | 115200 baud | DMA TX, interrupt RX |
| I2C1 | PB8 (SCL), PB9 (SDA) | 100 kHz | DMA RX, requires 4.7 kΩ pull-ups |
| TIM3 | — | 250 ms period | Triggers sensor read + LED toggle |
| SysTick | — | 1 ms tick | Used for timeouts and delays |

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No UART output | Baud rate 115200, PA2/PA3 wired, USART2 clock enabled in RCC |
| I2C not responding | 4.7 kΩ pull-ups on PB8/PB9, correct slave addresses (0x44 / 0x3F) |
| Build fails | `arm-none-eabi-gcc` in PATH, CMake ≥ 3.20, C++20 support |
| ST-Link not detected | USB drivers installed, 3.3 V target powered, SWD wired correctly |
| Sensor data all zeros | Verify sensor is powered and I2C ACKs are observed on a logic analyser |

## References

- [STM32F401xE Datasheet](Resources/STM32F401_Datasheet.pdf)
- [STM32F4 Reference Manual](Resources/STM32F401_Reference-Manual.pdf)
- [STM32F401 User Manual](Resources/STM32F401_User-Manual.pdf)
- [SHT40 Datasheet](https://sensirion.com/products/catalog/SHT40)
- [ARM Cortex-M4 Technical Reference Manual](https://developer.arm.com/documentation/100166/latest)

## Contributing

Pull requests are welcome for additional peripheral drivers (SPI, ADC, CAN), new sensor integrations, or PC-side packet parsers. Follow the two-layer driver pattern and keep all register access in the low-level headers.

## Author

**ahxiang99** — bare-metal embedded firmware engineering
