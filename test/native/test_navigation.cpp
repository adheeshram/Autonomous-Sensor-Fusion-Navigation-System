#include <cmath>
#include <cstdlib>
#include <iostream>

#include "navigation/DecisionEngine.h"
#include "navigation/FusionEngine.h"
#include "navigation/SensorFilter.h"

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

nav::RawMeasurement raw(float distance, bool valid = true, uint32_t time = 100) {
  return {nav::SensorId::UltrasonicLeft, nav::SensorKind::Ultrasonic,
          nav::Sector::Left, distance, time, valid};
}

nav::FilteredMeasurement filtered(nav::SensorId id,
                                  nav::SensorKind kind,
                                  nav::Sector sector,
                                  float distance,
                                  bool valid = true,
                                  float health = 1.0f,
                                  uint32_t time = 100) {
  return {id, kind, sector, distance, health, time, valid};
}

nav::FusedEstimate estimate(float distance, float confidence = 1.0f,
                            nav::Direction direction = nav::Direction::Center) {
  return {distance, distance, distance, confidence, 4, direction, true};
}

void testRobustFilterRejectsSpike() {
  nav::SensorFilter filter;
  filter.update(raw(100.0f));
  filter.update(raw(101.0f));
  filter.update(raw(99.0f));
  const nav::FilteredMeasurement spike = filter.update(raw(20.0f));
  expect(!spike.valid, "isolated spike should be rejected");
  expect(spike.health < 1.0f, "rejected sample should reduce sensor health");
}

void testHealthDegradesOnDropoutAndRecovers() {
  nav::SensorFilter filter;
  filter.update(raw(80.0f));
  for (int i = 0; i < 3; ++i) filter.update(raw(-1.0f, false));
  const float degraded = filter.health();
  expect(degraded < 0.5f, "repeated dropouts should degrade health");
  for (int i = 0; i < 8; ++i) filter.update(raw(80.0f));
  expect(filter.health() > degraded, "valid samples should restore health");
}

void testFusionSurvivesOneModalityDropout() {
  nav::FilteredMeasurement inputs[] = {
      filtered(nav::SensorId::UltrasonicLeft, nav::SensorKind::Ultrasonic,
               nav::Sector::Left, 24.0f),
      filtered(nav::SensorId::UltrasonicRight, nav::SensorKind::Ultrasonic,
               nav::Sector::Right, 75.0f),
      filtered(nav::SensorId::InfraredLeft, nav::SensorKind::Infrared,
               nav::Sector::Left, 25.0f),
      filtered(nav::SensorId::InfraredRight, nav::SensorKind::Infrared,
               nav::Sector::Right, 0.0f, false)};
  nav::FusionEngine engine;
  const nav::FusedEstimate result = engine.fuse(inputs, 4, 100);
  expect(result.valid, "fusion should remain valid with one sensor offline");
  expect(result.contributingSensors == 3, "only valid sensors should contribute");
  expect(result.direction == nav::Direction::Left, "closest occupied sector should be left");
  expect(result.distanceCm > 23.0f && result.distanceCm < 26.0f,
         "fused range should preserve the close obstacle");
}

void testDecisionHysteresis() {
  nav::DecisionEngine engine;
  nav::NavigationDecision decision = engine.update(estimate(20.0f), 100);
  expect(decision.level == nav::HazardLevel::Emergency,
         "unsafe range should enter emergency immediately");
  decision = engine.update(estimate(40.0f), 200);
  expect(decision.level == nav::HazardLevel::Emergency,
         "single safe-ish sample should not clear emergency");
  engine.update(estimate(40.0f), 300);
  decision = engine.update(estimate(40.0f), 400);
  expect(decision.level == nav::HazardLevel::Caution,
         "three recovery samples should lower emergency to caution");
}

void testClosingSpeedCanEscalateRisk() {
  nav::DecisionEngine engine;
  engine.update(estimate(150.0f), 100);
  const nav::NavigationDecision decision = engine.update(estimate(70.0f), 1100);
  expect(decision.closingSpeedCmPerSec > 10.0f, "approach rate should be estimated");
  expect(decision.level == nav::HazardLevel::Caution ||
             decision.level == nav::HazardLevel::Emergency,
         "short time-to-collision should escalate risk before distance threshold");
}

void testLowConfidenceIsFailSafe() {
  nav::DecisionEngine engine;
  const nav::NavigationDecision decision = engine.update(estimate(100.0f, 0.1f), 100);
  expect(decision.level == nav::HazardLevel::Degraded,
         "low-confidence estimate should request degraded mode");
  expect(decision.speedLimitPercent == 25, "degraded mode should cap commanded speed");
}

}  // namespace

int main() {
  testRobustFilterRejectsSpike();
  testHealthDegradesOnDropoutAndRecovers();
  testFusionSurvivesOneModalityDropout();
  testDecisionHysteresis();
  testClosingSpeedCanEscalateRisk();
  testLowConfidenceIsFailSafe();

  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All navigation core tests passed\n";
  return EXIT_SUCCESS;
}
