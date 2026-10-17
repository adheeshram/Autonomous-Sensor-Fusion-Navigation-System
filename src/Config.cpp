#include "navigation/Config.h"

namespace nav {

constexpr float Config::kMinDistanceCm;
constexpr float Config::kMaxDistanceCm;
constexpr float Config::kFilterAlpha;
constexpr float Config::kMinOutlierGateCm;
constexpr uint32_t Config::kStaleAfterMs;
constexpr float Config::kEmergencyDistanceCm;
constexpr float Config::kCautionDistanceCm;
constexpr float Config::kReleaseMarginCm;
constexpr float Config::kEmergencyTtcSeconds;
constexpr float Config::kCautionTtcSeconds;
constexpr uint8_t Config::kRecoveryCycles;
constexpr float Config::kMinimumOperationalConfidence;

}  // namespace nav
