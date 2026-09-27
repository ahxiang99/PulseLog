//
// Created by ahxia on 1/8/2026.
//

#ifndef BAREMETAL_HARDWARE_DEMO_TACHOMETER_H
#define BAREMETAL_HARDWARE_DEMO_TACHOMETER_H

#include <cstdint>

class FC51_IR
{
      public:
	struct config {
		TimerConfig timer_config;
		uint32_t timer_clock_hz;   // e.g. 84'000'000 (APB1 timer clock)
		uint32_t tick_freq_hz;     // e.g. 1'000'000 (1 MHz -> 1us/tick)
		float marks_per_rev;       // reflective marks on shaft, default 1
		uint32_t stall_timeout_ms; // e.g. 500ms -> report 0 RPM if exceeded
	} cfg_{};

	FC51_IR() = default;
	~FC51_IR() = default;
	Result<> initialize(const config &cfg = tach_cfg, const GPIO_Config &gpio_cfg = tim2_gpio_config);
	void onCaptureIRQ();
	void poll(uint32_t now_ms);
	[[nodiscard]] float get_rpm() const;

      protected:
      private:
	Stm32Timer timer_{};

	Result<> configureTimerBase_(const TimerConfig &cfg);
	void configureInputCapture_() const;
	volatile float rpm_{};
	volatile uint32_t last_capture_ms_{};
	volatile bool stalled_ = false;

	static constexpr config tach_cfg{
		.timer_config = {.Instance = TimerDevice_t::TIMER_2,
				 .AlignedMode = TimerCenterAlignedMode_t::EDGE,
				 .Direction = TimerDirection_t::UP,
				 .ClockDivision = TimerClockDivision_t::TIM_CLOCKDIVISION_DIV1,
				 .AutoReloadPreload = TimerARR_t::ENABLE},
		.timer_clock_hz = 84'000'000,
		.tick_freq_hz = 1'000'000,
		.marks_per_rev = 1,
		.stall_timeout_ms = 500,
	};
	static constexpr GPIO_Config tim2_gpio_config = {.pin = GPIO_PIN_0,
							 .port = GPIO_Port::GPIO_PA,
							 .mode = GPIO_Moder::GPIO_MODE_ALTFN,
							 .otype = GPIO_OType::GPIO_OTYPER_PP,
							 .ospdr = GPIO_OSPDR::GPIO_OSPEEDR_VHS,
							 .pupdr = GPIO_PUPDR::GPIO_PUPDR_PULLUP,
							 .afr = GPIO_AFR::GPIO_AF1_TIM1_2};
};

#endif // BAREMETAL_HARDWARE_DEMO_TACHOMETER_H
