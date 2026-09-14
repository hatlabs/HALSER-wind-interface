#ifndef WIND_INTERFACE_SRC_REFERENCE_ANGLE_H_
#define WIND_INTERFACE_SRC_REFERENCE_ANGLE_H_

// Pure, hardware-independent bound for the vane reference-angle recalibration,
// so it can be host-tested (pio test -e native). No Arduino/SensESP headers.

#include <cmath>

namespace wind_interface {

// Slack (radians) so an exact +/-180 deg entry still lands inside the range
// after the web UI's deg->rad conversion (displayMultiplier 180/pi) and its
// float rounding, rather than a hair over pi. ~0.00006 deg -- far above the
// round-trip error, far below any angle the sensor would treat as different.
inline constexpr double kReferenceAngleSlackRad = 1e-6;

// True if a reference angle (radians) is finite and within the Autonnic's
// +/-180 deg range. An out-of-range value is silently bounced by (and can stall)
// the sensor, so the firmware rejects it before commanding a recalibration.
inline bool reference_angle_in_range(double radians) {
  return std::isfinite(radians) &&
         std::fabs(radians) <= M_PI + kReferenceAngleSlackRad;
}

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_REFERENCE_ANGLE_H_
