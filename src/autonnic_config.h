// Autonnic A5120 configuration classes.
//
// Configuration is dual-stored: persisted to ESP32 filesystem (so values
// survive reboots) AND sent to the Autonnic as proprietary NMEA 0183 commands
// to synchronize the device state. This contrasts with the AIS interface where
// config lives only in the transponder.
//
// The save() pattern: persist to flash first, then build the NMEA sentence,
// send via UART, and wait for an ACK using SemaphoreValue with a timeout.
// The onDelay(0) wrapper defers the UART write to the event loop to avoid
// re-entrancy. WindOutputRepetitionRateConfig::save() is the exception: it
// sends directly (no onDelay wrapper) and uses a 5s timeout instead of 1s,
// because changing the repetition rate causes the Autonnic to pause output
// briefly before ACKing.
//
// AutonnicFloatConfig is a reusable base for any Autonnic config parameter
// that stores a single float. Each instance is parameterized with:
//   - a sentence builder function (constructs the proprietary NMEA sentence)
//   - a JSON key (for web UI serialization)
//   - a JSON schema string (for web UI form rendering)
// See main.cpp for usage examples.

#ifndef WIND_INTERFACE_SRC_AUTONNIC_CONFIG_H_
#define WIND_INTERFACE_SRC_AUTONNIC_CONFIG_H_

#include <elapsedMillis.h>
#include <sensesp/transforms/zip.h>

#include <tuple>

#include "ReactESP.h"
#include "autonnic_a5120_parser.h"
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
  /// @param nmea_io_task  NMEA 0183 I/O task used to send sentences
  /// @param default_value Default parameter value (used if no saved config)
  /// @param response_parser  Parser that emits on ACK from the Autonnic
  /// @param sentence_builder Function that constructs the NMEA command sentence
  /// @param json_key      JSON property name for web UI serialization
  /// @param config_schema JSON schema string for web UI form rendering
  /// @param config_path   SensESP filesystem path for persistent storage
  AutonnicFloatConfig(sensesp::nmea0183::NMEA0183IOTask* nmea_io_task,
                      float default_value,
                      AutonnicPATCWIMWVParser* response_parser,
                      SentenceBuilder sentence_builder,
                      const char* json_key, const char* config_schema,
                      String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
        value_{default_value},
        response_parser_{response_parser},
        sentence_builder_{sentence_builder},
        json_key_{json_key},
        config_schema_{config_schema} {
    load();
    response_parser_->connect_to(&response_semaphore_);
  }

  inline virtual bool to_json(JsonObject& doc) override {
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
    return this->FileSystemSaveable::load();
  }

  inline virtual bool save() override {
    this->FileSystemSaveable::save();
    String sentence = sentence_builder_(value_);
    ESP_LOGD("AutonnicFloatConfig", "Sending sentence: %s", sentence.c_str());
    response_semaphore_.clear();
    // Defer UART write to the event loop to avoid re-entrancy when save()
    // is called from within an event handler.
    sensesp::event_loop()->onDelay(
        0, [this, sentence]() { nmea_io_task_->set(sentence); });
    if (!response_semaphore_.take(1000)) {
      return false;
    }
    return true;
  }

  const char* get_config_schema() const { return config_schema_; }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
  float value_;
  AutonnicPATCWIMWVParser* response_parser_;
  SentenceBuilder sentence_builder_;
  const char* json_key_;
  const char* config_schema_;
  sensesp::SemaphoreValue<bool> response_semaphore_;
};

inline const String ConfigSchema(const AutonnicFloatConfig& obj) {
  return obj.get_config_schema();
}

class WindOutputRepetitionRateConfig : public sensesp::FileSystemSaveable,
                                       virtual public sensesp::Serializable {
 public:
  WindOutputRepetitionRateConfig(
      sensesp::nmea0183::NMEA0183IOTask* nmea_io_task, float repetition_rate,
      AutonnicPATCWIMWVParser* response_parser, String config_path = "")
      : sensesp::FileSystemSaveable(config_path),
        sensesp::Serializable(),
        nmea_io_task_{nmea_io_task},
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
    nmea_io_task_->set(sentence);
    if (!response_semaphore_.take(5000)) {
      ESP_LOGE("WindOutputRepetitionRate", "No response received");
      return false;
    }
    ESP_LOGV("WindOutputRepetitionRate", "Response received");
    return true;
  }

 protected:
  sensesp::nmea0183::NMEA0183IOTask* nmea_io_task_;
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
