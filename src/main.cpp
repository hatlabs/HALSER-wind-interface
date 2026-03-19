#include <Adafruit_NeoPixel.h>
#include <NMEA2000_esp32.h>

#include "Wire.h"
#include "autonnic_a5120_parser.h"
#include "autonnic_config.h"
#include "elapsedMillis.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/system/stream_producer.h"
#include "sensesp/transforms/filter.h"
#include "sensesp/transforms/typecast.h"
#include "sensesp/transforms/zip.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "sensesp_nmea0183/wiring.h"
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
  SetupLogging();

  Wire.setPins(kI2CSDAPin, kI2CSCLPin);
  Wire.begin();

  Serial1.begin(kWindBitRate, SERIAL_8N1, kUART1RxPin, kUART1TxPin);

  // SensESP application
  SensESPAppBuilder builder;
  auto sensesp_app = (&builder)
                         ->set_hostname("wind")
                         ->set_button_pin(kButtonPin)
                         ->enable_ota("thisisfine")
                         ->get_app();

  // RGB LED for activity indication
  led = new Adafruit_NeoPixel(1, kRGBLEDPin, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(30);

  NMEA0183IOTask* nmea0183_io_task = new NMEA0183IOTask(&Serial1);

  ApparentWindData* apparent_wind_data = new ApparentWindData();

  ConnectApparentWind(&(nmea0183_io_task->parser_), apparent_wind_data);

  // Connect the response parser
  AutonnicPATCWIMWVParser* autonnic_response_parser =
      new AutonnicPATCWIMWVParser(&(nmea0183_io_task->parser_));

  ReferenceAngleConfig* reference_angle_config = new ReferenceAngleConfig(
      nmea0183_io_task, 0, autonnic_response_parser, "/Wind/Reference Angle");

  ConfigItem(reference_angle_config)
      ->set_title("Reference Angle")
      ->set_description(
          "Reference angle offset for wind data (in degrees). "
          "Enter the angle readout when the wind vane is pointing "
          "straight ahead.")
      ->set_sort_order(300);

  WindDirectionDampingConfig* wind_direction_damping_config =
      new WindDirectionDampingConfig(nmea0183_io_task, 50.0,
                                     autonnic_response_parser,
                                     "/Wind/Direction Damping");

  ConfigItem(wind_direction_damping_config)
      ->set_title("Wind Direction Damping")
      ->set_description(
          "Wind direction damping factor (0-100.0). Default is "
          "50.0.")
      ->set_sort_order(400);

  WindSpeedDampingConfig* wind_speed_damping_config =
      new WindSpeedDampingConfig(nmea0183_io_task, 50.0,
                                 autonnic_response_parser,
                                 "/Wind/Speed Damping");

  ConfigItem(wind_speed_damping_config)
      ->set_title("Wind Speed Damping")
      ->set_description("Wind speed damping factor (0-100.0). Default is 50.0.")
      ->set_sort_order(500);

  WindOutputRepetitionRateConfig* wind_output_repetition_rate_config =
      new WindOutputRepetitionRateConfig(nmea0183_io_task, 500,
                                         autonnic_response_parser,
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

  // Reserve enough buffer for sending all messages.
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
  // Initialize NMEA 2000 wind data sender

  N2kWindDataSender* wind_data_sender = new N2kWindDataSender(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  apparent_wind_data->speed.connect_to(&(wind_data_sender->wind_speed_));

  apparent_wind_data->angle.connect_to(&(wind_data_sender->wind_angle_));

  wind_data_sender->connect_to(new LambdaConsumer<std::pair<double, double>>(
      [](std::pair<double, double> wind_data) {
        n2k_tx_counter = n2k_tx_counter.get() + 1;
        n2k_time_since_tx = 0;
      }));

  /////////////////////////////////////////////////////////////////////
  // Initialize the Signal K wind data sender

  auto apparent_wind_speed_sk_output = new SKOutputFloat(
      "/SK Path/Apparent Wind Speed", "environment.wind.speedApparent",
      new SKMetadata("Apparent Wind Speed", "m/s"));

  auto apparent_wind_angle_sk_output = new SKOutputFloat(
      "/SK Path/Apparent Wind Angle", "environment.wind.angleApparent",
      new SKMetadata("Apparent Wind Angle", "rad"));

  apparent_wind_data->speed.connect_to(apparent_wind_speed_sk_output);
  apparent_wind_data->angle.connect_to(apparent_wind_angle_sk_output);

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  CheckboxConfig* enable_n2k_watchdog_config = new CheckboxConfig(
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

  auto n2k_rx_ui_output = new StatusPageItem<int>("NMEA 2000 Received Messages",
                                                  0, "NMEA 2000", 300);

  n2k_rx_counter.connect_to(n2k_rx_ui_output);

  auto n2k_tx_ui_output = new StatusPageItem<int>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);

  n2k_tx_counter.connect_to(n2k_tx_ui_output);

  /////////////////////////////////////////////////////////////////////
  // Initialize the OLED display

  InfoDisplay* display = new InfoDisplay(&Wire);
  apparent_wind_data->speed.connect_to(
      &(display->apparent_wind_speed_consumer));
  apparent_wind_data->angle.connect_to(
      &(display->apparent_wind_angle_consumer));

  /////////////////////////////////////////////////////////////////////
  // LED: blink off briefly on each wind speed update

  apparent_wind_data->speed.connect_to(
      new LambdaConsumer<float>([](float) {
        led_off_until = millis() + 50;
      }));

  // LED animation + main loop
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
    event_loop()->tick();
  }
}

void loop() {}
