//
// Created by ahxia on 1/8/2026.
//

#include "FC51_IR.hpp"

#include "board_config.hpp"
#include "pch.hpp"

Result<> FC51_IR::initialize(const config &cfg, const GPIO_Config &gpio_cfg)
{
	if (!Gpio::configureMux(gpio_cfg).isOk()) {
		return Fail(Err::InvalidPinMask);
	}
	cfg_ = cfg;
	configureTimerBase_(cfg.timer_config);
	configureInputCapture_();

	const auto t = timer_.getInstance();
	RegisterUtils::setBits(t->DIER, TIM_DIER_CC1IE);
	RegisterUtils::setBits(t->CR1, TIM_CR1_CEN);
	return Ok();
}
void FC51_IR::onCaptureIRQ()
{
	if (auto t = timer_.getInstance(); t->SR & TIM_SR_CC1IF) {
		RegisterUtils::clearBits(t->SR, TIM_SR_CC1IF);
		uint32_t period_ticks = t->CCR1;
		if (period_ticks == 0) {
			return;
		}
		const float period_s = static_cast<float>(period_ticks) / static_cast<float>(cfg_.tick_freq_hz);
		const float rev_per_s = 1.0f / (period_s * cfg_.marks_per_rev);
		last_capture_ms_ = getDrivers().my_systick.get_ticks();
		rpm_ = rev_per_s * 60.0f;
		stalled_ = false;
	}
}
void FC51_IR::poll(uint32_t now_ms)
{
	if (!stalled_ && (now_ms - last_capture_ms_) > cfg_.stall_timeout_ms) {
		rpm_ = 0.0f;
		stalled_ = true;
	}
}
float FC51_IR::get_rpm() const
{
	return rpm_;
}

Result<> FC51_IR::configureTimerBase_(const TimerConfig &cfg)
{
	/* Enable Timer */
	if (!timer_.initialize(cfg).isOk()) {
		return Fail(Err::NullInstance);
	}

	const uint32_t psc = (cfg_.timer_clock_hz / cfg_.tick_freq_hz) - 1;
	const auto t = timer_.getInstance();
	t->PSC = psc;
	t->ARR = 0xFFFFFFFFu; // TIM2/TIM5 32-bit, free-run
	return Ok();
}
void FC51_IR::configureInputCapture_() const
{
	const auto t = timer_.getInstance();
	// IC1 <- TI1
	RegisterUtils::clearBits(t->CCMR1, 0x3U << 0);
	RegisterUtils::setBits(t->CCMR1, 0x1U << 0);

	// Input filter: tune N against expected max RPM
	RegisterUtils::clearBits(t->CCMR1, 0xFU << 4);
	RegisterUtils::setBits(t->CCMR1, 0x3U << 4);

	// Falling edge (active-low OUT), capture enable
	RegisterUtils::clearBits(t->CCER, 0x1U << 1);
	RegisterUtils::clearBits(t->CCER, 0x1U << 3);
	RegisterUtils::setBits(t->CCER, 0x1U << 1);
	RegisterUtils::setBits(t->CCER, 0x1U << 0);

	// Slave reset mode: counter resets to 0 on every capture edge,
	// so CCR1 directly holds period-in-ticks -- no manual delta/wraparound math.
	RegisterUtils::clearBits(t->SMCR, 0x7U << 0);
	RegisterUtils::clearBits(t->SMCR, 0x7U << 4);
	RegisterUtils::setBits(t->SMCR, 0x5U << 4);
	RegisterUtils::setBits(t->SMCR, 0x4U << 0);
}
