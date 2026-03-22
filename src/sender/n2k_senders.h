// Converts apparent wind data to NMEA 2000 PGN 130306 (Wind Data).
// Uses RepeatExpiring for wind_speed_ and wind_angle_: remembers the last
// received values but expires them after 5s of no input. Expired values are
// sent as N2kDoubleNA so the N2K bus always sees messages at the 100ms interval
// required by the NMEA 2000 standard.
// Also a ValueProducer: emits a (speed, angle) pair on each successful TX,
// allowing downstream consumers (like the TX counter) to track activity.

#ifndef WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_
#define WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <tuple>

#include "sensesp/system/expiring_value.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"
#include "sensesp/transforms/repeat.h"

namespace wind_interface {

/**
 * @brief Base class for NMEA 2000 senders.
 *
 */
class N2kSender : public sensesp::FileSystemSaveable,
                  virtual public sensesp::Serializable {
 public:
  N2kSender(String config_path)
      : sensesp::FileSystemSaveable{config_path}, sensesp::Serializable() {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_reaction_ != nullptr) {
      sensesp::event_loop()->remove(this->sender_reaction_);
      this->sender_reaction_ = nullptr;
    }
  }

 protected:
  reactesp::RepeatReaction* sender_reaction_ = nullptr;
};

class N2kWindDataSender
    : public N2kSender,
      public sensesp::ValueProducer<std::pair<double, double>> {
 public:
  N2kWindDataSender(String config_path, tN2kWindReference wind_reference,
                    tNMEA2000* nmea2000, bool enable = true)
      : N2kSender{config_path},
        wind_reference_{wind_reference},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000}           // In ms. When the inputs expire.
  {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            SetN2kWindSpeed(N2kMsg, 255, this->wind_speed_.get(),
                            this->wind_angle_.get(), this->wind_reference_);
            this->nmea2000_->SendMsg(N2kMsg);
            std::pair<double, double> wind_data = std::make_pair(
                this->wind_speed_.get(), this->wind_angle_.get());
            this->emit(wind_data);
          });
    }
  }

  // wind_angle_ and wind_speed_ depend on repeat_interval_ and expiry_
  // for initialization, but those are public API so we keep them all public.
  unsigned int repeat_interval_;
  unsigned int expiry_;

  sensesp::RepeatExpiring<double> wind_angle_{repeat_interval_, expiry_};
  sensesp::RepeatExpiring<double> wind_speed_{repeat_interval_, expiry_};

 protected:
  tNMEA2000* nmea2000_;
  tN2kWindReference wind_reference_;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_
