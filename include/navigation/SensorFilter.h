#pragma once

#include "navigation/Types.h"

namespace nav {

class SensorFilter {
 public:
  static constexpr uint8_t kWindowSize = 7;

  SensorFilter();
  FilteredMeasurement update(const RawMeasurement& measurement);
  float health() const { return health_; }

 private:
  float median(const float* values, uint8_t count) const;
  float medianAbsoluteDeviation(float center) const;

  float window_[kWindowSize];
  uint8_t count_;
  uint8_t cursor_;
  float smoothedCm_;
  float health_;
  bool initialized_;
};

}  // namespace nav
