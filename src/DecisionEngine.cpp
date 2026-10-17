#include "navigation/DecisionEngine.h"

#include "navigation/Config.h"

namespace nav {

DecisionEngine::DecisionEngine()
    : state_(HazardLevel::Degraded),
      recoveryCount_(0),
      previousDistanceCm_(0.0f),
      closingSpeedCmPerSec_(0.0f),
      previousTimestampMs_(0),
      hasPreviousEstimate_(false) {}

HazardLevel DecisionEngine::requestedState(const FusedEstimate& estimate,
                                           float ttcSeconds) const {
  if (!estimate.valid || estimate.confidence < Config::kMinimumOperationalConfidence) {
    return HazardLevel::Degraded;
  }
  if (estimate.distanceCm <= Config::kEmergencyDistanceCm ||
      (ttcSeconds > 0.0f && ttcSeconds <= Config::kEmergencyTtcSeconds)) {
    return HazardLevel::Emergency;
  }
  if (estimate.distanceCm <= Config::kCautionDistanceCm ||
      (ttcSeconds > 0.0f && ttcSeconds <= Config::kCautionTtcSeconds)) {
    return HazardLevel::Caution;
  }
  return HazardLevel::Clear;
}

NavigationDecision DecisionEngine::update(const FusedEstimate& estimate,
                                           uint32_t nowMs) {
  if (estimate.valid && hasPreviousEstimate_ && nowMs > previousTimestampMs_) {
    const float dt = static_cast<float>(nowMs - previousTimestampMs_) / 1000.0f;
    const float rawClosingSpeed = (previousDistanceCm_ - estimate.distanceCm) / dt;
    closingSpeedCmPerSec_ += 0.35f * (rawClosingSpeed - closingSpeedCmPerSec_);
    if (closingSpeedCmPerSec_ < 0.0f) closingSpeedCmPerSec_ = 0.0f;
  }

  if (estimate.valid) {
    previousDistanceCm_ = estimate.distanceCm;
    previousTimestampMs_ = nowMs;
    hasPreviousEstimate_ = true;
  }

  const float ttc = estimate.valid && closingSpeedCmPerSec_ > 1.0f
      ? estimate.distanceCm / closingSpeedCmPerSec_
      : -1.0f;
  const HazardLevel requested = requestedState(estimate, ttc);

  const bool moreSevere =
      (requested == HazardLevel::Emergency && state_ != HazardLevel::Emergency) ||
      (requested == HazardLevel::Caution && state_ == HazardLevel::Clear) ||
      (state_ == HazardLevel::Degraded && requested != HazardLevel::Clear);
  const bool forceDegraded = requested == HazardLevel::Degraded;
  if (moreSevere || forceDegraded) {
    state_ = requested;
    recoveryCount_ = 0;
  } else if (requested != state_) {
    bool beyondReleaseBoundary = true;
    if (state_ == HazardLevel::Emergency) {
      beyondReleaseBoundary = estimate.distanceCm >
          Config::kEmergencyDistanceCm + Config::kReleaseMarginCm;
    } else if (state_ == HazardLevel::Caution) {
      beyondReleaseBoundary = estimate.distanceCm >
          Config::kCautionDistanceCm + Config::kReleaseMarginCm;
    }

    if (beyondReleaseBoundary && ++recoveryCount_ >= Config::kRecoveryCycles) {
      state_ = requested;
      recoveryCount_ = 0;
    }
  } else {
    recoveryCount_ = 0;
  }

  uint8_t speedLimit = 100;
  bool alert = false;
  if (state_ == HazardLevel::Caution) {
    speedLimit = 45;
    alert = true;
  } else if (state_ == HazardLevel::Emergency) {
    speedLimit = 0;
    alert = true;
  } else if (state_ == HazardLevel::Degraded) {
    speedLimit = 25;
    alert = true;
  }

  const Direction direction = state_ == HazardLevel::Clear
      ? Direction::None : estimate.direction;
  return {state_, direction, estimate.distanceCm, closingSpeedCmPerSec_,
          ttc, speedLimit, alert};
}

}  // namespace nav
