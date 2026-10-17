#pragma once

#include "navigation/Types.h"

namespace nav {

class DecisionEngine {
 public:
  DecisionEngine();
  NavigationDecision update(const FusedEstimate& estimate, uint32_t nowMs);
  HazardLevel state() const { return state_; }

 private:
  HazardLevel requestedState(const FusedEstimate& estimate, float ttcSeconds) const;

  HazardLevel state_;
  uint8_t recoveryCount_;
  float previousDistanceCm_;
  float closingSpeedCmPerSec_;
  uint32_t previousTimestampMs_;
  bool hasPreviousEstimate_;
};

}  // namespace nav
