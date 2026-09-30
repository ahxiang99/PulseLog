#pragma once

#include "SHT40X.hpp"
#include "STTS22H.hpp"
#include "oled_disp.hpp"
#include "cpp/DmaI2C.hpp"
#include "cpp/ExtiInput.hpp"
#include "cpp/InterruptUart.hpp"
#include "cpp/Stm32Rcc.hpp"
#include "cpp/Stm32Spi.hpp"
#include "cpp/Stm32Timer.hpp"
#include "cpp/systick.hpp"
#include "cpp/wwdg.hpp"
#include "rtc/cpp/Stm32RTC.hpp"
#include "MAX30102.hpp"

template <typename T> struct peripherals_regs_table {
    T *instance;
    uint32_t enableBit;
    uint32_t resetBit;
};

struct DriversList {
    Stm32GpioPin gpio_led;
    InterruptUart uart2;
    DmaI2C i2c1;
    Stm32Spi spi1;
    MySysTick my_systick;
    Stm32Rtc rtc;
    Stm32Timer timer;
    SysClock sysclock;
    ExtiInput user_button;
    oled_disp disp;
    InterruptUart uart1;
    WindowWatchDog wwdg;
};

struct SensorsList {
    SHT40X SENSOR_SHT40X;
    STTS22H SENSOR_STTS22H;
    MAX30102 SENSOR_OXI;
    explicit SensorsList(const I2C_Ref &i2c) : SENSOR_SHT40X(i2c), SENSOR_STTS22H(i2c), SENSOR_OXI(i2c) {};
};

DriversList &getDrivers();

SensorsList &getSensors();
