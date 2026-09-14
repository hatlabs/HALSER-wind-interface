// HALSER Wind Interface Firmware — application entry point.
// Wires the data pipeline: Autonnic A5120 (NMEA 0183 over UART) → MWV
// sentence parser → N2K wind data sender (PGN 130306) + Signal K + OLED.
// Config objects (reference angle, damping, repetition rate) are dual-stored:
// ESP32 filesystem for persistence and Autonnic serial commands for device sync.

#include <NMEA2000_esp32.h>

#include <memory>

#include "Wire.h"
#include "autonnic_a5120_parser.h"
#include "autonnic_config.h"
#include "counting_nmea2000.h"
#include "elapsedMillis.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "ssd1306_display.h"

using namespace sensesp;
using namespace sensesp::nmea0183;
using namespace wind_interface;

// HALSER pin assignments
constexpr int kWindBitRate = 4800;
constexpr gpio_num_t kUART1RxPin = GPIO_NUM_3;
constexpr gpio_num_t kUART1TxPin = GPIO_NUM_2;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_4;
constexpr gpio_num_t kCANRxPin = GPIO_NUM_5;
constexpr int kI2CSDAPin = 6;
constexpr int kI2CSCLPin = 7;
constexpr int kButtonPin = 9;

ObservableValue<int> n2k_rx_counter = 0;

elapsedMillis n2k_time_since_rx = 0;

void setup() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  Wire.setPins(kI2CSDAPin, kI2CSCLPin);
  Wire.begin();

  // Enlarge the UART RX buffer before begin(): at 4800 bit/s the default
  // 256-byte buffer plus the 128-byte hardware FIFO holds about 100 ms of
  // traffic; 1024 bytes holds about 270 ms, giving the main loop comfortable
  // slack before bytes are dropped.
  Serial1.setRxBufferSize(1024);
  Serial1.begin(kWindBitRate, SERIAL_8N1, kUART1RxPin, kUART1TxPin);

  // SensESP application
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("wind")
                    ->set_button_pin(kButtonPin)
                    ->enable_ota("change-me")
                    ->get_app();

  // NMEA 0183 input, read on the main ReactESP loop. The ESP32-C3 is single-core
  // (CONFIG_FREERTOS_UNICORE), so a dedicated reader task buys no parallelism and
  // only adds a cross-task propagation hazard between the parser and its
  // main-loop consumers; NMEA0183IO reads on the event loop, keeping the whole
  // pipeline single-threaded.
  auto nmea_io = std::make_shared<NMEA0183IO>(&Serial1);

  // Sentence parsers self-register on the NMEA0183Parser.
  auto wind_parser = std::make_shared<MWVSentenceParser>(&nmea_io->parser_);
  auto autonnic_response_parser =
      std::make_shared<AutonnicPATCWIMWVParser>(&nmea_io->parser_);

  // Autonnic configuration parameters — each is an AutonnicFloatConfig
  // parameterized with a sentence builder, JSON key, and schema string.

  // The ±180° range is enforced in AutonnicReferenceAngleConfig::from_json, not
  // here: the SensESP number input ignores schema minimum/maximum, so adding
  // them would be dead config — don't rely on schema bounds for this field.
  auto reference_angle_config = std::make_shared<AutonnicReferenceAngleConfig>(
      &Serial1, autonnic_response_parser.get(),
      R"({"type":"object","properties":{"offset":{"title":"Angle in degrees","type":"number","displayMultiplier":57.29577951308232,"displayOffset":0}}})",
      "/Wind/Reference Angle");

  ConfigItem(reference_angle_config)
      ->set_title("Reference Angle")
      ->set_description(
          "Recalibrate the vane: enter the angle it should report for its "
          "current physical position, in degrees from -180 to 180 (0 = dead "
          "ahead, 180 = dead astern). Re-applying the same value is safe. This "
          "is a one-shot command: the entry is not stored and the field always "
          "reads 0.")
      ->set_sort_order(300);

  auto wind_direction_damping_config = std::make_shared<AutonnicFloatConfig>(
      &Serial1, 50.0, autonnic_response_parser.get(),
      AutonnicWindDirectionDampingSentence, "damping_factor",
      R"({"type":"object","properties":{"damping_factor":{"title":"Damping Factor","type":"number"}}})",
      "/Wind/Direction Damping");

  ConfigItem(wind_direction_damping_config)
      ->set_title("Wind Direction Damping")
      ->set_description(
          "Wind direction damping factor (0-100.0). Default is "
          "50.0.")
      ->set_sort_order(400);

  auto wind_speed_damping_config = std::make_shared<AutonnicFloatConfig>(
      &Serial1, 50.0, autonnic_response_parser.get(),
      AutonnicWindSpeedDampingSentence, "damping_factor",
      R"({"type":"object","properties":{"damping_factor":{"title":"Damping Factor","type":"number"}}})",
      "/Wind/Speed Damping");

  ConfigItem(wind_speed_damping_config)
      ->set_title("Wind Speed Damping")
      ->set_description("Wind speed damping factor (0-100.0). Default is 50.0.")
      ->set_sort_order(500);

  auto wind_output_repetition_rate_config =
      std::make_shared<WindOutputRepetitionRateConfig>(
          &Serial1, 500, autonnic_response_parser.get(),
          "/Wind/Message Repetition Rate");

  ConfigItem(wind_output_repetition_rate_config)
      ->set_title("Message Repetition Rate")
      ->set_description(
          "Wind message repetition rate in milliseconds. Default "
          "is 500.")
      ->set_sort_order(200);

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  CountingNMEA2000* nmea2000 = new CountingNMEA2000(kCANTxPin, kCANRxPin);

  // 64-frame CAN buffers: enough to absorb a fast-packet burst (up to 32 frames
  // each) while keeping the static footprint modest on the memory-constrained C3.
  nmea2000->SetN2kCANSendFrameBufSize(64);
  nmea2000->SetN2kCANReceiveFrameBufSize(64);

  nmea2000->SetProductInformation(
      "20240601",  // Manufacturer's Model serial code (max 32 chars)
      105,         // Manufacturer's product code
      "Wind-N2K",  // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number
      130,                     // Device function: Weather Instruments
      85,                      // Device class: Sensor Communication Interface
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 72);
  nmea2000->SetMsgHandler([](const tN2kMsg& msg) {
    n2k_rx_counter = n2k_rx_counter.get() + 1;
    n2k_time_since_rx = 0;
  });
  nmea2000->EnableForward(false);
  nmea2000->Open();

  event_loop()->onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 wind data sender

  auto wind_data_sender = std::make_shared<N2kWindDataSender>(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  // Wire wind parser outputs directly to N2K sender
  wind_parser->apparent_wind_speed_.connect_to(&(wind_data_sender->wind_speed_));
  wind_parser->apparent_wind_angle_.connect_to(&(wind_data_sender->wind_angle_));

  /////////////////////////////////////////////////////////////////////
  // Signal K outputs

  auto wind_speed_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed",
      new SKMetadata("m/s", "Apparent Wind Speed"));

  auto wind_angle_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle",
      new SKMetadata("rad", "Apparent Wind Angle"));

  wind_parser->apparent_wind_speed_.connect_to(wind_speed_sk);

  // Signal K's environment.wind.angleApparent is signed (negative to port). The
  // MWV parser emits 0..2pi, so map (pi, 2pi] to (-pi, 0]. The N2K PGN 130306
  // and OLED paths keep the unsigned 0..2pi range, so only this path is wrapped.
  auto wind_angle_to_signed = std::make_shared<LambdaTransform<float, float>>(
      [](float angle) {
        return angle > (float)M_PI ? angle - 2.0f * (float)M_PI : angle;
      });
  wind_parser->apparent_wind_angle_.connect_to(wind_angle_to_signed)
      ->connect_to(wind_angle_sk);

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  auto enable_n2k_watchdog_config = std::make_shared<CheckboxConfig>(
      false, "Enable NMEA 2000 Watchdog", "/NMEA2000/Enable Watchdog");

  ConfigItem(enable_n2k_watchdog_config)
      ->set_title("Enable NMEA 2000 Watchdog")
      ->set_description(
          "Enable the NMEA 2000 watchdog. If enabled, the device will reboot "
          "after two minutes if no NMEA 2000 messages are received. This "
          "setting requires a device restart to take effect.")
      ->set_requires_restart(true)
      ->set_sort_order(100);

  if (enable_n2k_watchdog_config->get_value()) {
    event_loop()->onRepeat(1000, [nmea2000]() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        delay(10);
        ESP.restart();
      }
    });
  }

  auto n2k_rx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Received Messages", 0, "NMEA 2000", 300);

  n2k_rx_counter.connect_to(n2k_rx_ui_output);

  auto n2k_tx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);

  nmea2000->tx_count_.connect_to(n2k_tx_ui_output);

  /////////////////////////////////////////////////////////////////////
  // OLED display

  auto display = std::make_shared<InfoDisplay>(&Wire);
  wind_parser->apparent_wind_speed_.connect_to(
      &(display->apparent_wind_speed_consumer));
  wind_parser->apparent_wind_angle_.connect_to(
      &(display->apparent_wind_angle_consumer));

  // Largest contiguous free block. This, not total free memory, gates large
  // allocations like the ~40 KB TLS handshake, so surface it alongside free
  // memory on the status page.
  auto largest_block_status = std::make_shared<StatusPageItem<int>>(
      "Largest free block (bytes)", 0, "System", 250);
  event_loop()->onRepeat(2000, [largest_block_status]() {
    largest_block_status->set(static_cast<int>(ESP.getMaxAllocHeap()));
  });

  // Main-loop task stack headroom: NMEA 0183 reading and parsing run on this
  // task, so this figure must stay well above zero. uxTaskGetStackHighWaterMark
  // returns the running task's minimum free stack in bytes on ESP-IDF.
  auto main_loop_stack_status = std::make_shared<StatusPageItem<int>>(
      "Main loop min free stack (bytes)", 0, "System", 260);
  event_loop()->onRepeat(2000, [main_loop_stack_status]() {
    main_loop_stack_status->set(
        static_cast<int>(uxTaskGetStackHighWaterMark(nullptr)));
  });

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
