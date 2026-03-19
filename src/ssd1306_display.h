#ifndef WIND_INTERFACE_SRC_SSD1306_DISPLAY_H_
#define WIND_INTERFACE_SRC_SSD1306_DISPLAY_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <sensesp/system/lambda_consumer.h>

#include <WiFi.h>

#include "sensesp_base_app.h"

namespace wind_interface {

// OLED display width and height, in pixels
const int kScreenWidth = 128;
const int kScreenHeight = 64;

class InfoDisplay {
 public:
  InfoDisplay(TwoWire* i2c) {
    display_ = new Adafruit_SSD1306(kScreenWidth, kScreenHeight, i2c, -1);
    bool init_successful = display_->begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (!init_successful) {
      ESP_LOGW("InfoDisplay", "SSD1306 allocation failed");
      return;
    }

    sensesp::event_loop()->onDelay(50, [this]() {
      display_->setRotation(2);
      display_->clearDisplay();
      display_->setTextSize(1);
      display_->setTextColor(SSD1306_WHITE);
      display_->setCursor(0, 0);

      display_->display();

      sensesp::event_loop()->onRepeat(1000, [this]() { update(); });
    });
  }

  sensesp::LambdaConsumer<float> apparent_wind_speed_consumer{
      [this](float value) { apparent_wind_speed_ = value; }};
  sensesp::LambdaConsumer<float> apparent_wind_angle_consumer{[this](float value) {
    // Convert radians to degrees and shift the range from 0 to 2pi to -180
    // to 180.
    this->apparent_wind_angle_ = value * 180 / M_PI;
    if (this->apparent_wind_angle_ > 180) {
      this->apparent_wind_angle_ -= 360;
    }
  }};

 private:
  Adafruit_SSD1306* display_;

  float apparent_wind_speed_ = 0;
  // Wind angle, in degrees from -180 to 180, where 0 is straight ahead.
  float apparent_wind_angle_ = 0;

  void clear_row(int row);
  void print_row(int row, String value);

  void update() {
    char row_buf[40];
    print_row(0, sensesp::SensESPBaseApp::get_hostname());
    print_row(1, WiFi.localIP().toString());
    snprintf(row_buf, sizeof(row_buf), "Uptime: %.0f s", millis() / 1000.0);
    print_row(2, row_buf);
    snprintf(row_buf, sizeof(row_buf), "AWS: %.1f m/s", apparent_wind_speed_);
    print_row(4, row_buf);
    snprintf(row_buf, sizeof(row_buf), "AWA: %.1f deg", apparent_wind_angle_);
    print_row(5, row_buf);
    display_->display();
  }
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_SSD1306_DISPLAY_H_
