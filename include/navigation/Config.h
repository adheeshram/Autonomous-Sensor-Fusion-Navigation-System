#pragma once

#include <stdint.h>

namespace nav {

struct Config {
  static constexpr float kMinDistanceCm = 4.0f;
  static constexpr float kMaxDistanceCm = 200.0f;
  static constexpr float kFilterAlpha = 0.45f;
  static constexpr float kMinOutlierGateCm = 12.0f;
  static constexpr uint32_t kStaleAfterMs = 250;

  static constexpr float kEmergencyDistanceCm = 25.0f;
  static constexpr float kCautionDistanceCm = 60.0f;
  static constexpr float kReleaseMarginCm = 8.0f;
  static constexpr float kEmergencyTtcSeconds = 0.8f;
  static constexpr float kCautionTtcSeconds = 2.5f;
  static constexpr uint8_t kRecoveryCycles = 3;
  static constexpr float kMinimumOperationalConfidence = 0.22f;
};

}  // namespace nav
