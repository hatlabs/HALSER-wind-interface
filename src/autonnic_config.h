// Autonnic A5120 configuration classes.
//
// Configuration is dual-stored: persisted to ESP32 filesystem (so values
// survive reboots) AND sent to the Autonnic as proprietary NMEA 0183 commands
// to synchronize the device state. This contrasts with the AIS interface where
// config lives only in the transponder.
//
// The save() pattern: persist to flash first, then build the NMEA sentence,
// send via UART, and wait for an ACK using SemaphoreValue with a timeout.
// save() runs on the HTTP server task; the onDelay(0) wrapper defers the UART
// write to the main event loop so it stays single-threaded with the NMEA reader
// (which also runs on the main loop), and so the blocking ACK wait never holds
// up the loop that has to parse that ACK. WindOutputRepetitionRateConfig uses a
// 5s timeout instead of 1s, because changing the repetition rate causes the
// Autonnic to pause output briefly before ACKing.
//
// AutonnicFloatConfig is a reusable base for any Autonnic config parameter
// that stores a single float. Each instance is parameterized with:
//   - a sentence builder function (constructs the proprietary NMEA sentence)
//   - a JSON key (for web UI serialization)
//   - a JSON schema string (for web UI form rendering)
// See main.cpp for usage examples.
//
// Set write_only for a one-shot command rather than a persistent setting (e.g.
// the vane reference recalibration, which tells the sensor what to read for its
// current physical position). A write-only parameter never echoes the last
// commanded value: to_json reports the neutral default_value and nothing is
// loaded or persisted, so the field always shows the default (e.g. 0) instead of
// a stale entry that could be mistaken for the sensor's current state.
// AutonnicReferenceAngleConfig below is the write-only specialization.

#ifndef WIND_INTERFACE_SRC_AUTONNIC_CONFIG_H_
#define WIND_INTERFACE_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>

#include <cmath>

#include "ReactESP.h"
#include "autonnic_a5120_parser.h"
#include "reference_angle.h"
#include "sensesp.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/semaphore_value.h"
#include "sensesp/system/serializable.h"
#include "sensesp_nmea0183/nmea0183.h"

namespace wind_interface {

// --- Sentence builders ---------------------------------------------------
// Each function constructs a proprietary Autonnic NMEA 0183 command sentence.
// These sentences intentionally omit checksums because the Autonnic A5120
// does not use them.

inline String AutonnicReferenceAngleSentence(const float& offset) {
  // $PATC,IIMWV,AHD,<degrees>
  float offset_degrees = offset * 180 / M_PI;
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,AHD,%0.1f", offset_degrees);
  return buf;
}

inline String AutonnicWindDirectionDampingSentence(const float& damping_factor) {
  // $PATC,IIMWV,DWD,<factor>
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,DWD,%0.1f", damping_factor);
  return buf;
}

inline String AutonnicWindSpeedDampingSentence(const float& damping_factor) {
  // $PATC,IIMWV,DSP,<factor>
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,DSP,%0.1f", damping_factor);
  return buf;
}

inline String AutonnicMessageRepetitionRateSentence(const int& repetition_rate) {
  // $PATC,IIMWV,TXP,<milliseconds>
  char buf[100];
  snprintf(buf, sizeof(buf), "$PATC,IIMWV,TXP,%d", repetition_rate);
  return buf;
}

// --- Reusable single-float config ----------------------------------------
// Covers any Autonnic parameter that is a single float value with the
// standard save() flow: persist → build sentence → send via event loop → wait
// for ACK. Parameterized at construction time so new float parameters can be
// added without writing another class.

/// Function type for building a proprietary NMEA sentence from a float value.
using SentenceBuilder = String (*)(const float&);

class AutonnicFloatConfig : public sensesp::FileSystemSaveable,
                            virtual public sensesp::Serializable {
 public:
  /// @param nmea_stream   Serial stream the command sentences are written to
  /// @param default_value Default parameter value (used if no saved config)
  /// @param response_parser  Parser that emits on ACK from the Autonnic
  /// @param sentence_builder Function that constructs the NMEA command sentence
  /// @param json_key      JSON property name for web UI serialization
  /// @param config_schema JSON schema string for web UI form rendering
  /// @param config_path   SensESP filesystem path for persistent storage
  /// @param write_only    If true, a one-shot command (not a stored setting):
  ///                      never persisted or loaded, and to_json always reports
  ///                      default_value instead of the last entry.
  AutonnicFloatConfig(Stream* nmea_stream,
                      float default_value,
                      AutonnicPATCWIMWVParser* response_parser,
                      SentenceBuilder sentence_builder,
                      const char* json_key, const char* config_schema,
                      String config_path = "", bool write_only = false)
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_stream_{nmea_stream},
        value_{default_value},
        default_value_{default_value},
        response_parser_{response_parser},
        sentence_builder_{sentence_builder},
        json_key_{json_key},
        config_schema_{config_schema},
        write_only_{write_only} {
    load();
    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    // A write-only parameter reports its neutral default, never the last
    // commanded value, so the field always reads the default (e.g. 0) rather
    // than echoing a one-shot command back as if it were stored state. The
    // entered value still reaches the device via from_json/save.
    if (write_only_) {
      doc[json_key_] = default_value_;
      return true;
    }
    doc[json_key_] = value_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    if (!config[json_key_].is<JsonVariant>()) {
      return false;
    }
    value_ = config[json_key_];
    return true;
  }

  inline virtual bool load() override {
    if (write_only_) {
      return true;  // no persisted shadow to restore
    }
    return this->FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    // A write-only command carries no state worth persisting (to_json reports a
    // fixed default and load() is skipped), so don't touch flash — only send the
    // sentence.
    if (!write_only_) {
      this->FileSystemSaveable::save();
    }
    String sentence = sentence_builder_(value_);
    ESP_LOGD("AutonnicFloatConfig", "Sending sentence: %s", sentence.c_str());
    response_semaphore_.clear();
    // Defer the UART write to the main loop (see file header).
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_stream_->println(sentence); });
    if (!response_semaphore_.take(1000)) {
      return false;
    }
    return true;
  }

  const char* get_config_schema() const { return config_schema_; }

 protected:
  Stream* nmea_stream_;
  float value_;
  float default_value_;
  AutonnicPATCWIMWVParser* response_parser_;
  SentenceBuilder sentence_builder_;
  const char* json_key_;
  const char* config_schema_;
  bool write_only_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const AutonnicFloatConfig& obj) {
  return obj.get_config_schema();
}

// The vane reference angle: a write-only, one-shot recalibration (the user
// enters the angle the vane should report for its current physical position).
// A "number" field defaulting to 0. from_json is overridden to enforce the
// Autonnic's own +/-180 deg range in firmware -- the SensESP number input does
// not honor schema minimum/maximum, and an out-of-range value is silently
// bounced by (and can stall) the sensor. The value arrives as radians (the web
// UI applies displayMultiplier 180/pi), stays radians to match the *180/pi in
// AutonnicReferenceAngleSentence, and is bounded to +/-pi.
class AutonnicReferenceAngleConfig : public AutonnicFloatConfig {
 public:
  AutonnicReferenceAngleConfig(Stream* nmea_stream,
                               AutonnicPATCWIMWVParser* response_parser,
                               const char* config_schema, String config_path)
      : AutonnicFloatConfig(nmea_stream, 0.0f, response_parser,
                            AutonnicReferenceAngleSentence, "offset",
                            config_schema, config_path, /*write_only=*/true) {}

  inline bool from_json(const JsonObject& config) override {
    auto value = config[json_key_];
    if (!value.is<float>() && !value.is<int>()) {
      return false;  // absent or not a number
    }
    double radians = value.as<double>();
    if (!reference_angle_in_range(radians)) {
      return false;  // outside +/-180 deg — don't command an out-of-range recal
    }
    value_ = radians;
    return true;
  }
};

inline const String ConfigSchema(const AutonnicReferenceAngleConfig& obj) {
  return obj.get_config_schema();
}

class WindOutputRepetitionRateConfig : public sensesp::FileSystemSaveable,
                                       virtual public sensesp::Serializable {
 public:
  WindOutputRepetitionRateConfig(
      Stream* nmea_stream, float repetition_rate,
      AutonnicPATCWIMWVParser* response_parser, String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_stream_{nmea_stream},
        repetition_rate_{repetition_rate},
        response_parser_{response_parser} {
    load();

    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
    doc["repetition_rate"] = repetition_rate_;
    return true;
  }

  inline virtual bool from_json(const JsonObject& config) override {
    String expected_keys[] = {"repetition_rate"};
    for (auto& key : expected_keys) {
      if (!config[key].is<JsonVariant>()) {
        return false;
      }
    }
    repetition_rate_ = config["repetition_rate"];

    return true;
  }

  inline virtual bool load() override {
    return FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    FileSystemSaveable::save();
    String sentence = AutonnicMessageRepetitionRateSentence(repetition_rate_);
    ESP_LOGD("WindOutputRepetitionRate", "Sending sentence: %s",
             sentence.c_str());
    response_semaphore_.clear();
    // Defer the UART write to the main loop (see file header).
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_stream_->println(sentence); });
    if (!response_semaphore_.take(5000)) {
      ESP_LOGE("WindOutputRepetitionRate", "No response received");
      return false;
    }
    ESP_LOGV("WindOutputRepetitionRate", "Response received");
    return true;
  }

 protected:
  Stream* nmea_stream_;
  float repetition_rate_;
  AutonnicPATCWIMWVParser* response_parser_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const WindOutputRepetitionRateConfig& obj) {
  const char schema[] = R"({
      "type": "object",
      "properties": {
        "repetition_rate": { "title": "Message Repetition Rate", "type": "integer" }
      }
    })";
  return schema;
}

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_AUTONNIC_CONFIG_H_
