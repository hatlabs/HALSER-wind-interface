// HALSER Wind Interface Firmware
// Autonnic A5120 wind instrument to NMEA 2000 gateway

#include <Adafruit_NeoPixel.h>
#include <NMEA2000_esp32.h>

#include <memory>

#include "Wire.h"
#include "autonnic_a5120_parser.h"
#include "autonnic_config.h"
#include "elapsedMillis.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/serial_number.h"
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
constexpr int kRGBLEDPin = 8;
constexpr int kButtonPin = 9;

static Adafruit_NeoPixel* led = nullptr;
static unsigned long led_off_until = 0;

ObservableValue<int> n2k_rx_counter = 0;
ObservableValue<int> n2k_tx_counter = 0;

elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

void setup() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  Wire.setPins(kI2CSDAPin, kI2CSCLPin);
  Wire.begin();

  Serial1.begin(kWindBitRate, SERIAL_8N1, kUART1RxPin, kUART1TxPin);

  // SensESP application
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("wind")
                    ->set_button_pin(kButtonPin)
                    ->enable_ota("thisisfine")
                    ->get_app();

  // RGB LED for activity indication
  led = new Adafruit_NeoPixel(1, kRGBLEDPin, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(30);

  // NMEA 0183 I/O task
  auto nmea0183_io_task = std::make_shared<NMEA0183IOTask>(&Serial1);

  // Wind sentence parser — connected directly to parser, no TaskQueueProducer
  // (ESP32-C3 is single-core, so cross-task bridging is unnecessary)
  auto wind_parser =
      std::make_shared<WIMWVSentenceParser>(&(nmea0183_io_task->parser_));

  // Autonnic response parser for configuration commands
  auto autonnic_response_parser =
      std::make_shared<AutonnicPATCWIMWVParser>(&(nmea0183_io_task->parser_));

  auto reference_angle_config = std::make_shared<ReferenceAngleConfig>(
      nmea0183_io_task.get(), 0, autonnic_response_parser.get(),
      "/Wind/Reference Angle");

  ConfigItem(reference_angle_config)
      ->set_title("Reference Angle")
      ->set_description(
          "Reference angle offset for wind data (in degrees). "
          "Enter the angle readout when the wind vane is pointing "
          "straight ahead.")
      ->set_sort_order(300);

  auto wind_direction_damping_config =
      std::make_shared<WindDirectionDampingConfig>(
          nmea0183_io_task.get(), 50.0, autonnic_response_parser.get(),
          "/Wind/Direction Damping");

  ConfigItem(wind_direction_damping_config)
      ->set_title("Wind Direction Damping")
      ->set_description(
          "Wind direction damping factor (0-100.0). Default is "
          "50.0.")
      ->set_sort_order(400);

  auto wind_speed_damping_config = std::make_shared<WindSpeedDampingConfig>(
      nmea0183_io_task.get(), 50.0, autonnic_response_parser.get(),
      "/Wind/Speed Damping");

  ConfigItem(wind_speed_damping_config)
      ->set_title("Wind Speed Damping")
      ->set_description("Wind speed damping factor (0-100.0). Default is 50.0.")
      ->set_sort_order(500);

  auto wind_output_repetition_rate_config =
      std::make_shared<WindOutputRepetitionRateConfig>(
          nmea0183_io_task.get(), 500, autonnic_response_parser.get(),
          "/Wind/Message Repetition Rate");

  ConfigItem(wind_output_repetition_rate_config)
      ->set_title("Message Repetition Rate")
      ->set_description(
          "Wind message repetition rate in milliseconds. Default "
          "is 500.")
      ->set_sort_order(200);

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  tNMEA2000* nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

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

  wind_data_sender->connect_to(
      std::make_shared<LambdaConsumer<std::pair<double, double>>>(
          [](std::pair<double, double>) {
            n2k_tx_counter = n2k_tx_counter.get() + 1;
            n2k_time_since_tx = 0;
          }));

  /////////////////////////////////////////////////////////////////////
  // Signal K outputs

  auto wind_speed_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed",
      new SKMetadata("m/s", "Apparent Wind Speed"));

  auto wind_angle_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle",
      new SKMetadata("rad", "Apparent Wind Angle"));

  wind_parser->apparent_wind_speed_.connect_to(wind_speed_sk);
  wind_parser->apparent_wind_angle_.connect_to(wind_angle_sk);

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

  n2k_tx_counter.connect_to(n2k_tx_ui_output);

  /////////////////////////////////////////////////////////////////////
  // OLED display

  auto display = std::make_shared<InfoDisplay>(&Wire);
  wind_parser->apparent_wind_speed_.connect_to(
      &(display->apparent_wind_speed_consumer));
  wind_parser->apparent_wind_angle_.connect_to(
      &(display->apparent_wind_angle_consumer));

  /////////////////////////////////////////////////////////////////////
  // LED: blink off briefly on each wind speed update

  wind_parser->apparent_wind_speed_.connect_to(
      std::make_shared<LambdaConsumer<float>>([](float) {
        led_off_until = millis() + 50;
      }));

  event_loop()->onRepeat(10, []() {
    if (millis() < led_off_until) {
      led->setPixelColor(0, 0);
    } else {
      uint16_t hue = (uint16_t)((millis() % 1000) * 65536UL / 1000);
      led->setPixelColor(0, led->ColorHSV(hue));
    }
    led->show();
  });

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
