#pragma once

#include <atomic>
#include <cstdint>

#include "cpp/II2C.hpp"
#include "drivers.hpp"
#include "logger.hpp"

struct PPGSample {
    uint32_t red;
    uint32_t ir;
};

class PPGBuffer
{
  public:
    void Push(const PPGSample &s);

    uint32_t DcRed() const;
    uint32_t DcIr() const;

  private:
    static constexpr size_t PPG_WINDOW = 100; // 1 second at 100Hz sample rate
    uint32_t Average(const uint32_t (&arr)[PPG_WINDOW]) const;

    uint32_t red_[PPG_WINDOW]{};
    uint32_t ir_[PPG_WINDOW]{};
    size_t idx_ = 0;
    size_t count_ = 0; // tracks how many slots are actually filled (handles
		       // startup, before buffer is full)
};

class HeartRateDetector
{
  public:
    bool Update(int32_t acIr, uint32_t timestampMs);

    uint32_t GetBpm() const;

  private:
    int32_t prevAc_ = 0;
    bool rising_ = false;
    bool armed_ = true;
    uint32_t lastBeatMs_ = 0;
    uint32_t bpm_ = 0;
    int32_t threshold_ = 250;
    int32_t resetThreshold_ = 200; // must drop below this before next beat counts — tune vs your data
    uint32_t minBeatIntervalMs_ = 350;
};

class AcAmplitudeTracker
{
  public:
    void Push(int32_t ac);

    int32_t GetAmplitude() const;

  private:
    static constexpr size_t WINDOW = 100; // ~1 second at 100Hz — spans a full beat
    int32_t max_ = INT32_MIN;
    int32_t min_ = INT32_MAX;
    int32_t amplitude_ = 0;
    size_t sampleCount_ = 0;
};

class SimpleSmoother
{
  public:
    int32_t Push(int32_t val);

  private:
    static constexpr size_t N = 4;
    int32_t buf_[N] = {};
    size_t idx_ = 0, count_ = 0;
};

class FingerDetector
{
  public:
    bool Update(uint32_t dcIr);

    bool IsPresent() const;

  private:
    static constexpr uint32_t kEnterThreshold = 5000;
    static constexpr uint32_t kExitThreshold = 3000;
    bool present_{false};
};

class MAX30102
{
  public:
    enum class SensorMode : uint8_t {
	HR = 2,
	SpO2
    };
    enum class FifoSampleAvg : uint8_t {
	NO_AVG,
	AVG2,
	AVG4,
	AVG8,
	AVG16,
	AVG32
    };
    enum class FifoRollOver : uint8_t {
	DISABLE,
	ENABLE
    };
    enum class SensorState : uint8_t {
	IDLE,
	WAIT_PTR,
	WAIT_DATA,
	DATA_READY
    };
    enum class SpO2ADC : uint8_t {
	SCALE_2048,
	SCALE_4096,
	SCALE_8192,
	SCALE_16384
    };
    enum class SpO2SampleRate : uint8_t {
	RATE_50,
	RATE_100,
	RATE_200,
	RATE_400,
	RATE_800,
	RATE_1000,
	RATE_1600,
	RATE_3200
    };
    enum class SpO2PulseWidth : uint8_t {
	ADC_15BITS,
	ADC_16BITS,
	ADC_17BITS,
	ADC_18BITS
    };

    struct SensorConfig {
	FifoSampleAvg sampleAvg;
	FifoRollOver rollOver;
	uint8_t FifoAlmostFullValue;
	SensorMode mode;
	SpO2ADC SpO2adc;
	SpO2SampleRate SpO2sr;
	SpO2PulseWidth SpO2pw;
    };

    struct SensorData_t {
	uint8_t bpm;
	float_t spo2;
    };

    MAX30102(const I2C_Ref &i2c);

    void init();

    uint8_t getPartID() const;

    void read();

    void processData();

    void onDataReceived();

    void setState(SensorState state);

    SensorState getState() const;

    uint8_t getSR1() const;

    bool getInit() const;

    void setConfig(const SensorConfig &c);

    bool isDataReady() const;

    void clearDataReadyFlag();

    SensorData_t getData() const;

    bool isFingerPresent() const;

  private:
    struct SensorConfig_t {
	uint8_t fifo;
	uint8_t mode;
	uint8_t SpO2;
    };

    PPGSample ParseFifoSample(const uint8_t buf[6]);

    float_t ComputeSpO2();

    bool init_{false};
    SensorConfig_t config_{};
    std::atomic<SensorState> state_{MAX30102::SensorState::IDLE};

    bool dataAvailable{false};

    I2C_Ref i2c_;

    HeartRateDetector hr;
    AcAmplitudeTracker redAmpTracker;
    AcAmplitudeTracker irAmpTracker;
    SimpleSmoother irSmoother;
    FingerDetector fingerDetector;

    uint8_t partID{};
    uint8_t IRQ_SR1{};
    uint8_t ptr_data_[3]{};

    PPGBuffer buf;
    SensorData_t processed_data{};
    uint8_t raw_data[6]{};

    uint32_t last_call_tick{0};

    static constexpr uint8_t kDevAddr = 0xAF;
    static constexpr uint8_t kIRQSR1Reg = 0x00;
    static constexpr uint8_t kFifoWritePtrReg = 0x04;
    static constexpr uint8_t kOverflowCounterReg = 0x05;
    static constexpr uint8_t kFifoReadPtrReg = 0x06;
    static constexpr uint8_t kFifoConfigReg = 0x08;
    static constexpr uint8_t kModeConfigReg = 0x09;
    static constexpr uint8_t kSpO2ConfigReg = 0x0A;
    static constexpr uint8_t kLED1ConfigReg = 0x0C;
    static constexpr uint8_t kLED2ConfigReg = 0x0D;
    static constexpr uint8_t kPartIDReg = 0xFF;
    static constexpr uint8_t kTimeOut = 3U;

    static constexpr uint8_t kFifoDR = 0x07;
    /* Pos */
    static constexpr uint8_t SME_AVE_Pos = 5;
    static constexpr uint8_t FIFO_ROLLOVER_EN_Pos = 4;
    static constexpr uint8_t FIFO_A_FULL_Pos = 0;
    static constexpr uint8_t MODE_Pos = 0;
    static constexpr uint8_t SPO2_LED_PW_Pos = 0;
    static constexpr uint8_t SPO2_SR_Pos = 2;
    static constexpr uint8_t SPO2_ADC_REG_Pos = 5;
};
