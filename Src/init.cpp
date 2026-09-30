// init.cpp
#include "Result.hpp"
#include "board_config.hpp"
#include "cpp/DmaUart.hpp"
#include "cpp/InterruptUart.hpp"
#include "cpp/Stm32GpioPin.hpp"
#include "drivers.hpp"
#include "logger.hpp"
#include "pch.hpp"
#include "cpp/InterruptI2C.hpp"

#include <typeinfo>
#include "AppMode.hpp"

extern std::atomic<AppMode> g_AppMode;

namespace
{

enum class BootTag : uint8_t {
    Clock = 1,
    Systick,
    GpioMux,
    Led,
    Uart2,
    Uart1,
    I2c1,
    Exti,
    Rtc,
    Spi1,
    Oled,
    Stts2h,
    Sht40x
};

template <typename T1, typename T2, typename T3, typename T4> void init_device_has_dma(T1 &dev, T2 &&cfg, T3 &&tx_cfg, T4 &&rx_cfg, BootTag tag)
{
    if constexpr (std::is_same_v<T1, DmaUart> || std::is_same_v<T1, DmaI2C>) {
	checkOk(dev.initialize(cfg, tx_cfg, rx_cfg), tag);
    } else if constexpr (std::is_same_v<T1, InterruptUart> || std::is_same_v<T1, InterruptI2C>) {
	checkOk(dev.initialize(cfg), tag);
    }
}

[[noreturn]] void panic(BootTag tag, uint32_t err)
{
    __disable_irq(); // no scheduler, no ISRs from here on

    // 1. Breadcrumb first — works even if UART is the casualty.
    _PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R[5] = 0xDEAD0000u | (static_cast<uint32_t>(tag) << 8) | (err & 0xFF);

    // 2. Best-effort raw UART. Polled TXE, no DMA, no Logger.
    //    Safe even pre-init: TE-not-set just means TXE never fires → timeout.
    auto putc = [](char c) {
	for (uint32_t t = 0; t < 50'000; ++t) { // bounded, never hangs
	    if (USART2->SR & USART_SR_TXE) {
		USART2->DR = c;
		return;
	    }
	}
    };
    const char msg[] = "\r\nPANIC tag=";
    for (const char *p = msg; *p; ++p) {
	putc(*p);
    }
    putc('0' + static_cast<uint8_t>(tag));

    auto pulse = [](bool on, uint32_t cycles) {
	GPIOA->BSRR = on ? GPIO_PIN_5 : (GPIO_PIN_5 << 16);
	for (volatile uint32_t i = 0; i < cycles; ++i) {
	}
    };

    // 3. SOS on LD2 via raw registers — no driver object needed.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOA_EN;
    (void)RCC->AHB1ENR;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 10)) | (1u << 10); // PA5 output
    constexpr uint32_t DOT = 150'000, DASH = 450'000, GAP = 450'000, WORD_GAP = 2'000'000;
    while (true) {
	for (int i = 0; i < 3; ++i) {
	    pulse(true, DOT);
	    pulse(false, GAP);
	} // S
	for (int i = 0; i < 3; ++i) {
	    pulse(true, DASH);
	    pulse(false, GAP);
	} // O
	for (int i = 0; i < 3; ++i) {
	    pulse(true, DOT);
	    pulse(false, GAP);
	} // S
	pulse(false, WORD_GAP);
    }
}

void checkOk(const Result<> &r, BootTag what)
{
    if (!r.isOk()) {
	panic(what, static_cast<uint32_t>(r.error()));
    }
}

void enableFpu()
{
    // Enable FPU by setting bits 20, 21, 22, and 23
    static volatile uint32_t &CPACR = *reinterpret_cast<volatile uint32_t *>(0xE000ED88);
    CPACR |= ((3UL << 20) | (3UL << 22));
    // Manual Barrier        instructions(Assembly)
    __asm volatile("dsb 0xf" ::: "memory");
    __asm volatile("isb 0xf" ::: "memory");
}

void initCore(DriversList &g)
{
    enableFpu();
    checkOk(g.sysclock.initialize(board::sys_cfg_84, board::clock_tree), BootTag::Clock);
    checkOk(g.my_systick.initialize(), BootTag::Systick);

    Logger::initialize(UartRef::from(g.uart2));
    Logger::set_level(LogLevel::INFO);
    LOG_PRINT("\n");
    LOG_INFO("AHB: {}, APB1: {}, APB2: {}", board::clock_tree.ahb, board::clock_tree.apb1, board::clock_tree.apb2);
}

void initComms(DriversList &g)
{
    init_device_has_dma(g.uart2, board::uart2::cfg, board::uart2::hdmatx_cfg, board::uart2::hdmarx_cfg, BootTag::Uart2);
    LOG_INFO("UART2 up");

    init_device_has_dma(g.uart1, board::uart1::cfg, nullptr, nullptr, BootTag::Uart1);
    init_device_has_dma(g.i2c1, board::i2c1::cfg, board::i2c1::config_tx, board::i2c1::config_rx, BootTag::I2c1);
    LOG_INFO("I2C1 up");
}

void initGpioMux()
{
    /* this is for peripherals gpio */
    static constexpr const GPIO_Config *mux[] = {&board::uart2::gpio_cfg, &board::uart1::gpio_cfg, &board::i2c1::gpio_cfg, &board::spi1::gpio_cfg};
    for (auto *cfg : mux) {
	checkOk(Gpio::configureMux(*cfg), BootTag::GpioMux);
    }
}

void initIo(DriversList &g)
{
    checkOk(g.gpio_led.initialize(board::led::gpio_cfg), BootTag::Led);
}

void initDisplay(DriversList &g)
{
    checkOk(g.spi1.initialize(board::spi1::cfg), BootTag::Spi1);
    LOG_INFO("SPI up");
    checkOk(g.disp.initialize(g.spi1, board::oled_scr::RST_CFG, board::oled_scr::DC_CFG, board::oled_scr::CS_CFG, g.my_systick), BootTag::Oled);
    LOG_INFO("OLED up");
}

void initTimer(DriversList &g)
{
    g.timer.initialize(board::tim3::cfg);
    LOG_INFO("TIM3 Initialized");
}

void initExti(DriversList &g)
{
    /* GPIO C Pin 13 */
    checkOk(g.user_button.initialize(board::exti::pc13_gpio_cfg, onButtonPress, &g_AppMode), BootTag::Exti);
    LOG_INFO("PC13 (Button) EXTI Up");
}

void initRtc(DriversList &g)
{
    /* Setup RTC Clock */
    if (!g.rtc.IsInit()) {
	RTC_TimeTypeDef t{.Hours = 15, .Minutes = 53, .Seconds = 0, .TimeFormat = 0};
	RTC_DateTypeDef d{.WeekDay = 1, .Year = 26, .Month = 7, .Day = 14};
	g.rtc.setTime(t);
	g.rtc.setDate(d);
    }
    checkOk(g.rtc.initialize(), BootTag::Rtc);
    LOG_INFO("RTC Up");
}

void initWatchdog(DriversList &g)
{
    g.wwdg.initialize(board::clock_tree.apb1);
    LOG_INFO("WWDG Up");
}

void logResetCause(const uint32_t csr)
{
    if (csr & RCC_CSR_WWDGRSTF) {
	LOG_WARN("Reset cause: Window Watchdog (sensor/loop lockup)");
    }
    if (csr & RCC_CSR_IWDGRSTF) {
	LOG_WARN("Reset cause: Independent Watchdog");
    }
    if (csr & RCC_CSR_BORRSTF) {
	LOG_WARN("Reset cause: Brown-out");
    }
    if (csr & RCC_CSR_PINRSTF) {
	LOG_INFO("Reset cause: NRST pin");
    }
    if (csr & RCC_CSR_PORRSTF) {
	LOG_INFO("Reset cause: Power-on");
    }
    if (csr & RCC_CSR_SFTRSTF) {
	LOG_INFO("Reset cause: Software reset");
    }
    if (csr & RCC_CSR_LPWRRSTF) {
	LOG_WARN("Reset cause: Low-power mode exit");
    }
}

void logPanicBreadcrumb(uint32_t breadcrumb)
{
    if ((breadcrumb & 0xFFFF0000u) != 0xDEAD0000u) {
	return;
    }
    const auto tag = static_cast<BootTag>((breadcrumb >> 8) & 0xFF);
    const uint32_t err = breadcrumb & 0xFF;
    LOG_ERROR("Previous boot panicked: tag={}, err={}", static_cast<uint32_t>(tag), err);
}

void logBootDiagnostics(uint32_t csr, uint32_t breadcrumb)
{
    logResetCause(csr);
    logPanicBreadcrumb(breadcrumb);

    /* Clear breadcrumb and reset-cause flags so the next boot's read is clean */
    _PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R[5] = 0;
    RegisterUtils::setBits(RCC->CSR, RCC_CSR_RMVF);
}

} // namespace

void initDriver(DriversList &g)
{
    /* Snapshot reset-cause and last panic breadcrumb before anything clears them */
    const uint32_t bootCsr = RCC->CSR;
    const uint32_t bootBreadcrumb = RTC->BKP0R[5];

    initCore(g);
    initGpioMux();
    initComms(g);
    logBootDiagnostics(bootCsr, bootBreadcrumb);
    initIo(g);
    initExti(g);
    initTimer(g);
    initRtc(g);
    initDisplay(g);
#if defined(WATCH_DOG_ENABLE)
    initWatchdog(g);
#endif
    LOG_INFO("Boot complete");
}

void initSensor(SensorsList &g)
{
    // Link Sensor to i2c
    getDrivers().i2c1.addReceiver(g.SENSOR_SHT40X);
    getDrivers().i2c1.addReceiver(g.SENSOR_STTS22H);
#if defined(MAX30102_OXI_EN)
    getDrivers().i2c1.addReceiver(g.SENSOR_OXI);
    MAX30102::SensorConfig oxi_config{MAX30102::FifoSampleAvg::AVG4,      MAX30102::FifoRollOver::ENABLE,      0, MAX30102::SensorMode::SpO2, MAX30102::SpO2ADC::SCALE_4096,
				      MAX30102::SpO2SampleRate::RATE_100, MAX30102::SpO2PulseWidth::ADC_18BITS};
    g.SENSOR_OXI.setConfig(oxi_config);

    if (!g.SENSOR_OXI.getInit()) {
	g.SENSOR_OXI.init();
	LOG_INFO("Part ID: {}", static_cast<uint16_t>(g.SENSOR_OXI.getPartID()));
    }
#endif
    checkOk(g.SENSOR_SHT40X.initialize(SHT40X::Command::HIGH_PRECISION), BootTag::Sht40x);
    checkOk(g.SENSOR_STTS22H.initialize(STTS22H::SensorMode::ODR_01HZ), BootTag::Stts2h);
}
