#include "navigation/SensorFilter.h"

#include <math.h>

#include "navigation/Config.h"

namespace nav {

SensorFilter::SensorFilter()
    : window_{0.0f},
      count_(0),
      cursor_(0),
      smoothedCm_(0.0f),
      health_(1.0f),
      initialized_(false) {}

float SensorFilter::median(const float* values, uint8_t count) const {
  float sorted[kWindowSize];
  for (uint8_t i = 0; i < count; ++i) sorted[i] = values[i];
  for (uint8_t i = 1; i < count; ++i) {
    const float value = sorted[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }
  if ((count & 1U) != 0U) return sorted[count / 2];
  return 0.5f * (sorted[count / 2 - 1] + sorted[count / 2]);
}

float SensorFilter::medianAbsoluteDeviation(float center) const {
  float deviations[kWindowSize];
  for (uint8_t i = 0; i < count_; ++i) deviations[i] = fabsf(window_[i] - center);
  return median(deviations, count_);
}

FilteredMeasurement SensorFilter::update(const RawMeasurement& measurement) {
  const bool inRange = measurement.valid &&
      measurement.distanceCm >= Config::kMinDistanceCm &&
      measurement.distanceCm <= Config::kMaxDistanceCm;

  if (!inRange) {
    health_ *= 0.72f;
    if (health_ < 0.0f) health_ = 0.0f;
    return {measurement.id, measurement.kind, measurement.sector, smoothedCm_,
            health_, measurement.timestampMs, false};
  }

  bool accepted = true;
  if (count_ >= 3) {
    const float center = median(window_, count_);
    const float robustSigma = 1.4826f * medianAbsoluteDeviation(center);
    const float gate = robustSigma * 3.0f > Config::kMinOutlierGateCm
        ? robustSigma * 3.0f
        : Config::kMinOutlierGateCm;
    accepted = fabsf(measurement.distanceCm - center) <= gate;
  }

  if (!accepted) {
    health_ *= 0.85f;
    return {measurement.id, measurement.kind, measurement.sector, smoothedCm_,
            health_, measurement.timestampMs, false};
  }

  window_[cursor_] = measurement.distanceCm;
  cursor_ = (cursor_ + 1U) % kWindowSize;
  if (count_ < kWindowSize) ++count_;

  const float robustReading = median(window_, count_);
  if (!initialized_) {
    smoothedCm_ = robustReading;
    initialized_ = true;
  } else {
    smoothedCm_ += Config::kFilterAlpha * (robustReading - smoothedCm_);
  }
  health_ += 0.08f * (1.0f - health_);

  return {measurement.id, measurement.kind, measurement.sector, smoothedCm_,
          health_, measurement.timestampMs, true};
}

}  // namespace nav
