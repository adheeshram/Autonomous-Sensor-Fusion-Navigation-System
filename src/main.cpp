#include <Arduino.h>

#include "navigation/DecisionEngine.h"
#include "navigation/FusionEngine.h"
#include "navigation/SensorFilter.h"

namespace {

constexpr uint8_t kSensorCount = static_cast<uint8_t>(nav::SensorId::Count);
constexpr uint8_t kTrigPins[2] = {2, 4};
constexpr uint8_t kEchoPins[2] = {3, 5};
constexpr uint8_t kIrPins[2] = {A0, A1};
constexpr uint8_t kBuzzerPin = 9;
constexpr uint8_t kLedPin = 10;
constexpr uint32_t kCyclePeriodMs = 50;
constexpr uint32_t kEchoTimeoutUs = 12000;

nav::SensorFilter filters[kSensorCount];
nav::FusionEngine fusionEngine;
nav::DecisionEngine decisionEngine;
uint32_t lastCycleMs = 0;

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  const unsigned long durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  return durationUs == 0 ? -1.0f : durationUs * 0.0343f * 0.5f;
}

float readInfraredCm(uint8_t pin) {
  const int adc = analogRead(pin);
  if (adc <= 3) return -1.0f;
  return 6787.0f / static_cast<float>(adc - 3) - 4.0f;
}

nav::RawMeasurement raw(nav::SensorId id,
                        nav::SensorKind kind,
                        nav::Sector sector,
                        float distance,
                        uint32_t timestampMs) {
  return {id, kind, sector, distance, timestampMs, distance > 0.0f};
}

void applyOutputs(const nav::NavigationDecision& decision) {
  digitalWrite(kLedPin, decision.alertEnabled ? HIGH : LOW);
  if (decision.level == nav::HazardLevel::Emergency) {
    tone(kBuzzerPin, 1400);
  } else if (decision.level == nav::HazardLevel::Caution) {
    tone(kBuzzerPin, 850, 35);
  } else if (decision.level == nav::HazardLevel::Degraded) {
    tone(kBuzzerPin, 500, 20);
  } else {
    noTone(kBuzzerPin);
  }
}

void publishTelemetry(const nav::FilteredMeasurement* sensors,
                      const nav::FusedEstimate& estimate,
                      const nav::NavigationDecision& decision,
                      uint32_t timestampMs) {
  Serial.print(F("{\"t_ms\":")); Serial.print(timestampMs);
  Serial.print(F(",\"sensors_cm\":["));
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (i != 0) Serial.print(',');
    if (sensors[i].valid) Serial.print(sensors[i].distanceCm, 1);
    else Serial.print(F("null"));
  }
  Serial.print(F("],\"health\":["));
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (i != 0) Serial.print(',');
    Serial.print(sensors[i].health, 2);
  }
  Serial.print(F("],\"distance_cm\":")); Serial.print(estimate.distanceCm, 1);
  Serial.print(F(",\"confidence\":")); Serial.print(estimate.confidence, 2);
  Serial.print(F(",\"state\":\"")); Serial.print(nav::toString(decision.level));
  Serial.print(F("\",\"direction\":\"")); Serial.print(nav::toString(decision.direction));
  Serial.print(F("\",\"ttc_s\":"));
  if (decision.timeToCollisionSec > 0.0f) Serial.print(decision.timeToCollisionSec, 2);
  else Serial.print(F("null"));
  Serial.print(F(",\"speed_limit_pct\":")); Serial.print(decision.speedLimitPercent);
  Serial.println('}');
}

}  // namespace

void setup() {
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(kTrigPins[i], OUTPUT);
    pinMode(kEchoPins[i], INPUT);
    pinMode(kIrPins[i], INPUT);
  }
  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kLedPin, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("{\"event\":\"boot\",\"system\":\"sensor-fusion-nav\",\"version\":\"3.0.0\"}"));
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastCycleMs < kCyclePeriodMs) return;
  lastCycleMs = nowMs;

  const nav::RawMeasurement readings[kSensorCount] = {
      raw(nav::SensorId::UltrasonicLeft, nav::SensorKind::Ultrasonic,
          nav::Sector::Left, readUltrasonicCm(kTrigPins[0], kEchoPins[0]), nowMs),
      raw(nav::SensorId::UltrasonicRight, nav::SensorKind::Ultrasonic,
          nav::Sector::Right, readUltrasonicCm(kTrigPins[1], kEchoPins[1]), nowMs),
      raw(nav::SensorId::InfraredLeft, nav::SensorKind::Infrared,
          nav::Sector::Left, readInfraredCm(kIrPins[0]), nowMs),
      raw(nav::SensorId::InfraredRight, nav::SensorKind::Infrared,
          nav::Sector::Right, readInfraredCm(kIrPins[1]), nowMs)};

  nav::FilteredMeasurement filtered[kSensorCount];
  for (uint8_t i = 0; i < kSensorCount; ++i) filtered[i] = filters[i].update(readings[i]);

  const nav::FusedEstimate estimate = fusionEngine.fuse(filtered, kSensorCount, nowMs);
  const nav::NavigationDecision decision = decisionEngine.update(estimate, nowMs);
  applyOutputs(decision);
  publishTelemetry(filtered, estimate, decision, nowMs);
}
