#include "pch.hpp"

#include "app/AppMode.hpp"
#include "app/init.hpp"
#include "app/main_globals.hpp"
#include "cli.hpp"
#include "cpp/ExtiInput.hpp"
#include "cpp/II2C.hpp"
#include "cpp/Stm32GpioPin.hpp"
#include "cpp/UartRef.hpp"
#include "FloatIntExtraction.hpp"
#include "MAX30102.hpp"
#include "RingBuffer.hpp"
#include "sensors/SensorPacket.hpp"
#include "SHT40X.hpp"
#include "STTS22H.hpp"

// Selects the packet format sent in AppMode::SendPacket — the framed binary
// Packet<T, PacketType> protocol (console/PC tooling) vs. the raw
// Env_Sensor_Data struct (legacy wire format). See appModeOperation().
#define CONSOLE_APP

// Referenced by Src/init.cpp (extern std::atomic<AppMode> g_AppMode;), which
// wires it into the PC13 button's EXTI callback context — must keep external
// linkage and this exact name.
std::atomic<AppMode> g_AppMode{AppMode::Console};

namespace
{

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

Max30102 g_oxi_sensor(I2C_Ref::from(getDrivers().i2c1));
Cli g_cli;

std::atomic_bool g_cmd_complete{true};

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */

void registerCallbacks();
void queueSensorReads(SpscRingBuffer<I2CCommand, 4> &cmd_queue, const AppMode &cur_mode, SensorsList &sensor_list);
void runOximeter(DriversList &g);
void serviceWatchdog(DriversList &g, SensorsList &sensor_list, uint32_t &wwdg_refresh_start);
void appModeOperation(DriversList &g, SensorsList &sensor_list, uint32_t &measure_start);
void oledScreenUpdate(DriversList &g, SensorsList &sensor_list, uint32_t &disp_start);

/* Pushes a sensor read command onto the queue and logs (rather than
   silently dropping) if the queue is full — a full queue means the I2C
   consumer has stalled for multiple ticks and is worth surfacing. */
void pushCommand(SpscRingBuffer<I2CCommand, 4> &cmd_queue, void (*fn)(void *), void *ctx, const char *sensor_name)
{
    if (!cmd_queue.push({.fn = fn, .ctx = ctx})) {
	LOG_WARN("cmd_queue full, dropped {} read", sensor_name);
    }
}

} // namespace

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */

int main()
{
    DriversList &g = getDrivers();
    SensorsList &sensor_list = getSensors();

    registerCallbacks();
    initDriver(g);
    initSensor(sensor_list);

    if (!g.timer.isRunning()) {
	g.timer.start(20);
    }

    SpscRingBuffer<I2CCommand, 4> cmd_queue;

    if constexpr (kOxiMeterEnable) {
	Max30102::SensorConfig oxi_config{Max30102::FifoSampleAvg::AVG4,      Max30102::FifoRollOver::ENABLE,      0, Max30102::SensorMode::SpO2, Max30102::SpO2ADC::SCALE_4096,
					  Max30102::SpO2SampleRate::RATE_100, Max30102::SpO2PulseWidth::ADC_18BITS};
	g_oxi_sensor.setConfig(oxi_config);

	if (!g_oxi_sensor.getInit()) {
	    g_oxi_sensor.init();
	    LOG_INFO("Part ID: {}", static_cast<uint16_t>(g_oxi_sensor.getPartID()));
	}
    }

    uint32_t measure_start = g.my_systick.get_ticks();
    uint32_t disp_start = g.my_systick.get_ticks();
    uint32_t wwdg_refresh_start = g.my_systick.get_ticks();

    while (true) {
	const AppMode cur_mode = g_AppMode.load(std::memory_order_relaxed);

	g.i2c1.processRx();
	g.user_button.processEvent();

	serviceWatchdog(g, sensor_list, wwdg_refresh_start);

	if (g_cmd_execute.load(std::memory_order_relaxed)) {
	    g.gpio_led.toggle();
	    queueSensorReads(cmd_queue, cur_mode, sensor_list);
	    g_cmd_execute.store(false, std::memory_order_relaxed);
	}

	if constexpr (kSensorEnable) {
	    if constexpr (kOxiMeterEnable) {
		pushCommand(cmd_queue, [](void *ctx) { static_cast<Max30102 *>(ctx)->read(); }, &g_oxi_sensor, "oximeter");
		runOximeter(g);
	    }

	    oledScreenUpdate(g, sensor_list, disp_start);
	    appModeOperation(g, sensor_list, measure_start);
	}

	if (g_cmd_complete.load(std::memory_order_acquire)) {
	    I2CCommand cmd{};
	    if (cmd_queue.pop(cmd)) {
		g_cmd_complete.store(false, std::memory_order_relaxed);
		g.i2c1.complete_flag_ = &g_cmd_complete;
		cmd.fn(cmd.ctx);
	    }
	}

	/* Sleep until the next interrupt (SysTick/DMA/UART/EXTI) instead of busy-spinning */
	__WFI();
    }
}

/* ------------------------------------------------------------------ */
/* Setup                                                               */
/* ------------------------------------------------------------------ */

namespace
{

void registerCallbacks()
{
    getDrivers().uart2.onDataReceived([](void *ctx, const uint8_t *data, size_t len) { static_cast<Cli *>(ctx)->onUartData(data, len); }, &g_cli);
    g_cli.setUart(UartRef::from(getDrivers().uart2));

    if constexpr (kSensorEnable) {
	getDrivers().i2c1.addReceiver(g_cli);
	if constexpr (kOxiMeterEnable) {
	    getDrivers().i2c1.addReceiver(g_oxi_sensor);
	}
    }
}

/* ------------------------------------------------------------------ */
/* Main-loop helpers                                                   */
/* ------------------------------------------------------------------ */

/* Fires every 5 x 20ms tick (set by TIM3_IRQHandler, see Src/isr.cpp).
   Skipped in CLI mode so a queued read doesn't clobber CLI framing. */
void queueSensorReads(SpscRingBuffer<I2CCommand, 4> &cmd_queue, const AppMode &cur_mode, SensorsList &sensor_list)
{
    if (cur_mode == AppMode::CliMode) {
	return;
    }
    pushCommand(cmd_queue, [](void *ctx) { static_cast<SHT40X *>(ctx)->read(); }, &sensor_list.SENSOR_SHT40X, "SHT40X");
    pushCommand(cmd_queue, [](void *ctx) { static_cast<STTS22H *>(ctx)->read(); }, &sensor_list.SENSOR_STTS22H, "STTS22H");
}

void runOximeter(DriversList &g)
{
    g_oxi_sensor.processData();

    char buf[128];
    if (g_oxi_sensor.isDataReady()) {
	const FloatIntExtraction spo2 = convertInt(g_oxi_sensor.getData().spo2);
	snprintf(buf, sizeof(buf), "BPM and SpO2 Found.");
	g.disp.show(buf, 0, 0, 0);
	snprintf(buf, sizeof(buf), "BPM: %d, SpO2: %d.%d", g_oxi_sensor.getData().bpm, spo2.Integer, spo2.Decimal);
	g.disp.show(buf, 0, 8, 1);
	g_oxi_sensor.clearDataReadyFlag();
    }

    if (!g_oxi_sensor.isFingerPresent()) {
	snprintf(buf, sizeof(buf), "Finger not Found.");
	g.disp.show(buf, 0, 0, 0);
	g.disp.clear_page(1);
	g.disp.flush_page(1);
    }
}

/* Feeds the watchdog on its own cadence, inside the ~29-50ms window the
   configured prescaler/window/reload actually allows a refresh (the WWDG's
   window is far shorter than the 100ms sensor-poll timer below, so it must
   not be gated on that). Skipped while the sensor reports a fault so a
   stuck I2C bus lets the WWDG time out and reset the board. */
void serviceWatchdog(DriversList &g, SensorsList &sensor_list, uint32_t &wwdg_refresh_start)
{
    if (g.my_systick.get_ticks() - wwdg_refresh_start <= 35) {
	return;
    }
    if (sensor_list.SENSOR_SHT40X.getFaultStatus().isOk()) {
	g.wwdg.resetCounter();
    }
    wwdg_refresh_start = g.my_systick.get_ticks();
}

void appModeOperation(DriversList &g, SensorsList &s, uint32_t &measure_start)
{
    switch (g_AppMode.load(std::memory_order_relaxed)) {
    case AppMode::Console: {
	if (g.my_systick.get_ticks() - measure_start > 1000) {
	    LOG_INFO("SHT40: Temp: {} C, Rh: {}", s.SENSOR_SHT40X.getValue().temperature, s.SENSOR_SHT40X.getValue().humidity);
	    LOG_INFO("STTS2H: Temp: {}", s.SENSOR_STTS22H.getTemp());
	    measure_start = g.my_systick.get_ticks();
	}
	break;
    }
    case AppMode::SendPacket: {
	if (g.my_systick.get_ticks() - measure_start > 350) {
#if defined CONSOLE_APP
	    const Packet<SHT40X::SensorData, PacketType::PKT_SHT40> pkt_sht40_data{s.SENSOR_SHT40X.getValue()};
	    g.uart2.send({pkt_sht40_data.raw(), pkt_sht40_data.size()});
	    const Packet<float_t, PacketType::PKT_STTS2H> pkt_stts2h_data{s.SENSOR_STTS22H.getTemp()};
	    g.uart2.send({pkt_stts2h_data.raw(), pkt_stts2h_data.size()});
#else
	    static uint16_t seq = 0;
	    const Env_Sensor_Data sht40_data{
		    .last_rx_tick = static_cast<uint16_t>(g.my_systick.get_ticks() - measure_start),
		    .seq = seq++,
		    .temp_x100 = static_cast<int16_t>(s.SENSOR_SHT40X.getValue().temperature * 100),
		    .rh_x100 = static_cast<int16_t>(s.SENSOR_SHT40X.getValue().humidity * 100),
	    };
	    const Packet<Env_Sensor_Data, PacketType::PKT_ENV_SENSOR_DATA> pkt_sht40_data{sht40_data};
	    g.uart1.send({pkt_sht40_data.raw(), pkt_sht40_data.size()});
#endif
	    measure_start = g.my_systick.get_ticks();
	}
	break;
    }
    case AppMode::CliMode: {
	if (g_cli.getState() == CliState::Completed) {
	    LOG_INFO("STH40: Temp:{}, Rh:{}", s.SENSOR_SHT40X.getValue().temperature, s.SENSOR_SHT40X.getValue().humidity);
	    g_cli.setState(CliState::WaitingForInput);
	}
	g.uart2.processRx();
	if (g_cli.getState() == CliState::WaitingForInput) {
	    g_cli.get_input();
	    g_cli.setState(CliState::Processing);
	}
	break;
    }
    default:
	break;
    }
}

void oledScreenUpdate(DriversList &g, SensorsList &s, uint32_t &disp_start)
{
    char buf[128];

    RTC_DateTypeDef d = g.rtc.getDate();
    RTC_TimeTypeDef t = g.rtc.getTime();
    snprintf(buf, sizeof(buf), "20%02d/%02d/%02d %02d:%02d:%02d", d.Year, d.Month, d.Day, t.Hours, t.Minutes, t.Seconds);
    g.disp.show(buf, 0, 0, 0);

    snprintf(buf, sizeof(buf), "AppMode: %s", namingTable[static_cast<uint8_t>(g_AppMode.load(std::memory_order_relaxed))].name);
    g.disp.show(buf, 0, 8, 1);

    if (g.my_systick.get_ticks() - disp_start > 500) {
	FloatIntExtraction temp = convertInt(s.SENSOR_SHT40X.getValue().temperature);
	snprintf(buf, sizeof(buf), "SHT40: Temp:%02d.%02d", temp.Integer, temp.Decimal);
	g.disp.show(buf, 0, 16, 2);
	temp = convertInt(s.SENSOR_SHT40X.getValue().humidity);
	snprintf(buf, sizeof(buf), "SHT40: Rh:%02d.%02d", temp.Integer, temp.Decimal);
	g.disp.show(buf, 0, 24, 3);
	temp = convertInt(s.SENSOR_STTS22H.getTemp());
	snprintf(buf, sizeof(buf), "STTS2H: Temp:%02d.%02d", temp.Integer, temp.Decimal);
	g.disp.show(buf, 0, 32, 4);
	disp_start = g.my_systick.get_ticks();
    }
}

} // namespace
