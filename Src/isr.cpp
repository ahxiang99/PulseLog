// All interrupt vector handlers for the application live here. ISR bodies
// stay minimal by design: clear/service the peripheral, forward to the
// owning driver, and (for TIM3) set a flag for the main loop to act on.
// See CLAUDE.md "ISR routing" — no HAL weak-symbol overrides, handlers
// delegate straight into the driver layer.

#include "pch.hpp"

#include "app/main_globals.hpp"
#include "drivers.hpp"

std::atomic_bool g_cmd_execute{true};

namespace {

template <typename Driver> inline void handleTxDmaIfSupported(Driver &driver)
{
    if constexpr (Driver::kHasDma) {
        driver.handleTxDmaInterrupt();
    }
}

template <typename Driver> inline void handleRxDmaIfSupported(Driver &driver)
{
    if constexpr (Driver::kHasDma) {
        driver.handleRxDmaInterrupt();
    }
}

} // namespace

extern "C" void TIM3_IRQHandler(void)
{
    getDrivers().timer.handleInterrupt();

    if (getSensors().SENSOR_SHT40X.getState() == SHT40X::SensorState::MEASURING) {
        getSensors().SENSOR_SHT40X.processData();
    }

    static uint8_t count = 0;
    count++;
    if (count == 5) {
        g_cmd_execute.store(true, std::memory_order_relaxed);
        count = 0;
    }
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
    getDrivers().i2c1.handleRxDmaInterrupt();
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
    getDrivers().i2c1.handleTxDmaInterrupt();
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
    handleTxDmaIfSupported(getDrivers().uart2);
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
    handleRxDmaIfSupported(getDrivers().uart2);
}

extern "C" void USART1_IRQHandler(void)
{
    getDrivers().uart1.handleInterrupt();
}

extern "C" void USART2_IRQHandler(void)
{
    getDrivers().uart2.handleInterrupt();
}

extern "C" void I2C1_EV_IRQHandler(void)
{
    getDrivers().i2c1.handleEVInterrupt();
}

extern "C" void I2C1_ER_IRQHandler(void)
{
    getDrivers().i2c1.handleERInterrupt();
}

// Unused today: no GPIO on pins 5-9 is currently configured, so
// EXTI9_5_IRQn is never enabled in the NVIC (see drivers/exti/cpp/ExtiInput.cpp,
// which only enables the line for the pin actually configured). If a future
// pin in that range is wired up, this handler MUST clear the corresponding
// EXTI->PR bit before returning — an empty handler on a live interrupt line
// re-triggers immediately on return and livelocks the core.
extern "C" void EXTI9_5_IRQHandler(void)
{
}

extern "C" void EXTI15_10_IRQHandler(void)
{
    getDrivers().user_button.handleInterrupt();
}

// Unused today: TIM2 is not started anywhere, so TIM2_IRQn is never enabled.
// Same caveat as EXTI9_5_IRQHandler above applies if it ever is.
extern "C" void TIM2_IRQHandler(void)
{
}

extern "C" void HardFault_Handler(void)
{
    /* Put a breakpoint on the line below */
    volatile int loop = 1;
    while (loop) {
        // If the debugger stops here, a HardFault occurred!
    }
}
