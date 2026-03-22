// Parses proprietary ACK responses ($PATC,WIMWV,ACK) from the Autonnic A5120
// wind instrument. ignore_checksum(true) because the Autonnic does not include
// NMEA checksums on its responses.

#ifndef WIND_INTERFACE_SRC_AUTONNIC_A5120_PARSER_H_
#define WIND_INTERFACE_SRC_AUTONNIC_A5120_PARSER_H_

#include "sensesp.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/field_parsers.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace wind_interface {

class AutonnicPATCWIMWVParser : public sensesp::nmea0183::SentenceParser {
 public:
  AutonnicPATCWIMWVParser(sensesp::nmea0183::NMEA0183Parser *nmea0183)
      : SentenceParser(nmea0183) {
    ignore_checksum(true);
  }

  inline virtual const char *sentence_address() override final { return "PATC,WIMWV"; }

  inline bool parse_fields(const char *field_strings, const int field_offsets[],
                    int num_fields) override final {
    String response;

    // Example sentence: $PATC,WIMWV,ACK

    if (num_fields != 3) {
      return false;
    }

    bool ok = sensesp::nmea0183::ParseString(&response,
                                             field_strings + field_offsets[2]);

    response_.set(response);

    ESP_LOGV("AutonnicPATCWIMWVParser", "Response: %s", response.c_str());

    emit(ok);
    return ok;
  }

  sensesp::ObservableValue<String> response_;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_AUTONNIC_A5120_PARSER_H_
