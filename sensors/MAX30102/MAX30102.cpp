#include "MAX30102.hpp"
void PPGBuffer::Push(const PPGSample &s)
{
    red_[idx_] = s.red;
    ir_[idx_] = s.ir;
    idx_ = (idx_ + 1) % PPG_WINDOW;
    if (count_ < PPG_WINDOW) {
	count_++;
    }
}
uint32_t PPGBuffer::DcRed() const
{
    return Average(red_);
}
uint32_t PPGBuffer::DcIr() const
{
    return Average(ir_);
}
uint32_t PPGBuffer::Average(const uint32_t (&arr)[PPG_WINDOW]) const
{
    uint64_t sum = 0; // 64-bit accumulator: avoid overflow across 100 samples
    for (size_t i = 0; i < count_; i++) {
	sum += arr[i];
    }
    return count_ ? (uint32_t)(sum / count_) : 0;
}
bool HeartRateDetector::Update(int32_t acIr, uint32_t timestampMs)
{
    bool beatDetected = false;

    if (!armed_ && acIr < resetThreshold_) {
	armed_ = true; // signal has dropped low enough to re-arm for the next peak
    }

    if (acIr > prevAc_ && !rising_) {
	rising_ = true;
    } else if (acIr < prevAc_ && rising_) {
	rising_ = false;
	if (armed_ && prevAc_ > threshold_ && // use the actual peak value
	    (timestampMs - lastBeatMs_) > minBeatIntervalMs_) {
	    if (lastBeatMs_ != 0) {
		uint32_t interval = timestampMs - lastBeatMs_;
		bpm_ = 60000 / interval;
		beatDetected = true;
	    }
	    lastBeatMs_ = timestampMs;
	    armed_ = false; // don't allow another beat until we drop back down
	}
    }
    prevAc_ = acIr;
    return beatDetected;
}
uint32_t HeartRateDetector::GetBpm() const
{
    return bpm_;
}
void AcAmplitudeTracker::Push(int32_t ac)
{
    if (ac > max_) {
	max_ = ac;
    }
    if (ac < min_) {
	min_ = ac;
    }
    sampleCount_++;

    if (sampleCount_ >= WINDOW) {
	amplitude_ = max_ - min_;
	max_ = INT32_MIN;
	min_ = INT32_MAX;
	sampleCount_ = 0;
    }
}
int32_t AcAmplitudeTracker::GetAmplitude() const
{
    return amplitude_;
}
int32_t SimpleSmoother::Push(int32_t val)
{
    buf_[idx_] = val;
    idx_ = (idx_ + 1) % N;
    if (count_ < N) {
	count_++;
    }
    int32_t sum = 0;
    for (size_t i = 0; i < count_; i++) {
	sum += buf_[i];
    }
    return sum / (int32_t)count_;
}
bool FingerDetector::Update(uint32_t dcIr)
{
    if (!present_ && dcIr > kEnterThreshold) {
	present_ = true;
    } else if (present_ && dcIr < kExitThreshold) {
	present_ = false;
    }
    return present_;
}
bool FingerDetector::IsPresent() const
{
    return present_;
}
void MAX30102::init()
{
    /* Need to Write Once to power on the sensor -> Will show NACK */
    i2c_.MemRead(kDevAddr, kIRQSR1Reg, &IRQ_SR1, 1, kTimeOut);
    getDrivers().my_systick.delay_ms(10);
    /* The actual setting start here */
    init_ = false;
    /* Write FIFO Config */
    if (!i2c_.MemWrite(kDevAddr, kFifoConfigReg, &config_.fifo, sizeof(config_.mode), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    /* Write Mode Config */
    if (!i2c_.MemWrite(kDevAddr, kModeConfigReg, &config_.mode, sizeof(config_.mode), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    /* Write SpO2 Config */
    if (!i2c_.MemWrite(kDevAddr, kSpO2ConfigReg, &config_.SpO2, sizeof(config_.SpO2), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    /* Write LED1 */
    uint8_t Byte = 0x24;
    if (!i2c_.MemWrite(kDevAddr, kLED1ConfigReg, &Byte, sizeof(Byte), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    /* Write LED2 */
    if (!i2c_.MemWrite(kDevAddr, kLED2ConfigReg, &Byte, sizeof(Byte), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);

    /* Clear FIFO Pointers */
    Byte = 0;
    if (!i2c_.MemWrite(kDevAddr, kFifoWritePtrReg, &Byte, sizeof(Byte), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    if (!i2c_.MemWrite(kDevAddr, kOverflowCounterReg, &Byte, sizeof(Byte), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    if (!i2c_.MemWrite(kDevAddr, kFifoReadPtrReg, &Byte, sizeof(Byte), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);
    /* Get Part ID */
    if (!i2c_.MemRead(kDevAddr, kPartIDReg, &partID, sizeof(partID), kTimeOut)) {
	return;
    }
    getDrivers().my_systick.delay_ms(5);

    getDrivers().i2c1.processRx();
    init_ = true;
}
MAX30102::MAX30102(const I2C_Ref &i2c) : i2c_(i2c)
{
}
uint8_t MAX30102::getPartID() const
{
    return partID;
}
void MAX30102::read()
{
    if (state_.load(std::memory_order_relaxed) == SensorState::IDLE) {
	if (i2c_.MemRead(kDevAddr, kFifoWritePtrReg, (uint8_t *)ptr_data_, sizeof(ptr_data_), kTimeOut)) {
	    setState(SensorState::WAIT_PTR);
	    last_call_tick = getDrivers().my_systick.get_ticks();
	}
    }
}
void MAX30102::processData()
{
    if (state_.load(std::memory_order_relaxed) == SensorState::DATA_READY) {
	PPGSample data = ParseFifoSample(raw_data);
	buf.Push(data);
	if (!fingerDetector.Update(buf.DcIr())) {
	    setState(SensorState::IDLE);
	    return; // no finger — skip AC/HR/SpO2 processing entirely
	}
	int32_t acRed = (int32_t)data.red - (int32_t)buf.DcRed();
	int32_t acIr = (int32_t)data.ir - (int32_t)buf.DcIr();
	int32_t smoothedIr = irSmoother.Push(acIr);
	/* Print BPM */
	constexpr int32_t MAX_PLAUSIBLE_AC = 3000;
	if (std::abs(acRed) < MAX_PLAUSIBLE_AC && std::abs(acIr) < MAX_PLAUSIBLE_AC) {
	    redAmpTracker.Push(acRed);
	    irAmpTracker.Push(acIr);

	    if (hr.Update(smoothedIr, getDrivers().my_systick.get_ticks())) {
		processed_data.bpm = hr.GetBpm();
		processed_data.spo2 = ComputeSpO2();
		if (processed_data.bpm >= 40 && processed_data.bpm <= 180) {
		    dataAvailable = true;
		    LOG_INFO("BPM: {}  SpO2: {}", (uint16_t)processed_data.bpm, processed_data.spo2);
		}
	    }
	}

	setState(SensorState::IDLE);
    }
}
void MAX30102::onDataReceived()
{
    SensorState s = state_.load(std::memory_order_relaxed);

    if (s == SensorState::WAIT_PTR) {
	uint8_t wrPtr = ptr_data_[0];
	uint8_t rdPtr = ptr_data_[2]; // OVF_COUNTER sits in between, ptr_data_[1]

	if (wrPtr != rdPtr) {
	    // genuinely new data waiting — issue the real FIFO burst read
	    if (i2c_.MemRead(kDevAddr, kFifoDR, raw_data, sizeof(raw_data), kTimeOut)) {
		setState(SensorState::WAIT_DATA);
	    } else {
		setState(SensorState::IDLE);
	    }
	} else {
	    setState(SensorState::IDLE); // nothing new yet, try again next cycle
	}
    } else if (s == SensorState::WAIT_DATA) {
	setState(SensorState::DATA_READY);
    }
}
void MAX30102::setState(SensorState state)
{
    state_.store(state, std::memory_order_relaxed);
}
MAX30102::SensorState MAX30102::getState() const
{
    return state_.load(std::memory_order_relaxed);
}
uint8_t MAX30102::getSR1() const
{
    return IRQ_SR1;
}
bool MAX30102::getInit() const
{
    return init_;
}
void MAX30102::setConfig(const SensorConfig &c)
{
    config_.fifo = (static_cast<uint8_t>(c.sampleAvg) << SME_AVE_Pos) | (static_cast<uint8_t>(c.rollOver) << FIFO_ROLLOVER_EN_Pos) | (c.FifoAlmostFullValue << FIFO_A_FULL_Pos);
    config_.mode = static_cast<uint8_t>(c.mode) << MODE_Pos;
    config_.SpO2 = (static_cast<uint8_t>(c.SpO2adc) << SPO2_ADC_REG_Pos) | (static_cast<uint8_t>(c.SpO2sr) << SPO2_SR_Pos) | (static_cast<uint8_t>(c.SpO2pw) << SPO2_LED_PW_Pos);
}
bool MAX30102::isDataReady() const
{
    return dataAvailable;
}
void MAX30102::clearDataReadyFlag()
{
    dataAvailable = false;
}
MAX30102::SensorData_t MAX30102::getData() const
{
    return processed_data;
}
bool MAX30102::isFingerPresent() const
{
    return fingerDetector.IsPresent();
}
PPGSample MAX30102::ParseFifoSample(const uint8_t buf[6])
{
    PPGSample s;
    s.red = ((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2]) & 0x3FFFF;
    s.ir = ((uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | buf[5]) & 0x3FFFF;
    return s;
}
float_t MAX30102::ComputeSpO2()
{
    int32_t acAmpRed = redAmpTracker.GetAmplitude();
    int32_t acAmpIr = irAmpTracker.GetAmplitude();
    uint32_t dcRed = buf.DcRed();
    uint32_t dcIr = buf.DcIr();

    if (dcRed == 0 || dcIr == 0 || acAmpIr == 0) {
	return -1.0f; // guard against divide-by-zero, sensor not ready / no signal
    }

    float_t ratioRed = (float)acAmpRed / (float)dcRed;
    float_t ratioIr = (float)acAmpIr / (float)dcIr;
    float_t R = ratioRed / ratioIr;

    float_t spo2 = 110.0f - 25.0f * R;

    if (spo2 > 100.0f) {
	spo2 = 100.0f;
    }
    if (spo2 < 0.0f) {
	spo2 = 0.0f;
    }

    return spo2;
}
