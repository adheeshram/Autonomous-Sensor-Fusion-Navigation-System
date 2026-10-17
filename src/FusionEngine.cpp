#include "navigation/FusionEngine.h"

#include <math.h>

#include "navigation/Config.h"

namespace nav {

FusionEngine::SectorEstimate FusionEngine::fuseSector(
    const FilteredMeasurement* measurements,
    uint8_t count,
    Sector sector,
    uint32_t nowMs) const {
  float weightedDistance = 0.0f;
  float totalWeight = 0.0f;
  uint8_t contributors = 0;

  for (uint8_t i = 0; i < count; ++i) {
    const FilteredMeasurement& m = measurements[i];
    if (!m.valid || m.sector != sector || nowMs - m.timestampMs > Config::kStaleAfterMs) continue;

    const float modalityWeight = m.kind == SensorKind::Ultrasonic ? 1.0f : 0.8f;
    const float ageRatio = static_cast<float>(nowMs - m.timestampMs) /
                           static_cast<float>(Config::kStaleAfterMs);
    const float freshness = 1.0f - 0.5f * ageRatio;
    const float weight = modalityWeight * m.health * freshness;
    weightedDistance += m.distanceCm * weight;
    totalWeight += weight;
    ++contributors;
  }

  if (contributors == 0 || totalWeight <= 0.0f) return {0.0f, 0.0f, 0, false};
  return {weightedDistance / totalWeight, totalWeight, contributors, true};
}

FusedEstimate FusionEngine::fuse(const FilteredMeasurement* measurements,
                                 uint8_t count,
                                 uint32_t nowMs) const {
  const SectorEstimate left = fuseSector(measurements, count, Sector::Left, nowMs);
  const SectorEstimate right = fuseSector(measurements, count, Sector::Right, nowMs);

  if (!left.valid && !right.valid) {
    return {0.0f, 0.0f, 0.0f, 0.0f, 0, Direction::Unknown, false};
  }

  const float leftDistance = left.valid ? left.distanceCm : 0.0f;
  const float rightDistance = right.valid ? right.distanceCm : 0.0f;
  const float distance = !left.valid ? rightDistance
      : !right.valid ? leftDistance
      : leftDistance < rightDistance ? leftDistance : rightDistance;

  Direction direction = Direction::None;
  if (left.valid && right.valid) {
    const float difference = fabsf(leftDistance - rightDistance);
    direction = difference <= 10.0f ? Direction::Center
        : leftDistance < rightDistance ? Direction::Left : Direction::Right;
  } else {
    direction = left.valid ? Direction::Left : Direction::Right;
  }

  const uint8_t contributors = left.contributors + right.contributors;
  float confidence = (left.weight + right.weight) / 3.6f;
  if (left.valid && right.valid) {
    const float disagreement = fabsf(leftDistance - rightDistance);
    const float agreementFactor = 1.0f - (disagreement / Config::kMaxDistanceCm);
    confidence *= agreementFactor > 0.55f ? agreementFactor : 0.55f;
  } else {
    confidence *= 0.65f;
  }
  if (confidence > 1.0f) confidence = 1.0f;

  return {distance, leftDistance, rightDistance, confidence, contributors,
          direction, true};
}

}  // namespace nav
