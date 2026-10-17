#pragma once

#include <stdint.h>

namespace nav {

enum class SensorId : uint8_t {
  UltrasonicLeft = 0,
  UltrasonicRight,
  InfraredLeft,
  InfraredRight,
  Count
};

enum class SensorKind : uint8_t { Ultrasonic, Infrared };
enum class Sector : uint8_t { Left, Right };
enum class Direction : uint8_t { None, Left, Center, Right, Unknown };
enum class HazardLevel : uint8_t { Clear, Caution, Emergency, Degraded };

struct RawMeasurement {
  SensorId id;
  SensorKind kind;
  Sector sector;
  float distanceCm;
  uint32_t timestampMs;
  bool valid;
};

struct FilteredMeasurement {
  SensorId id;
  SensorKind kind;
  Sector sector;
  float distanceCm;
  float health;
  uint32_t timestampMs;
  bool valid;
};

struct FusedEstimate {
  float distanceCm;
  float leftDistanceCm;
  float rightDistanceCm;
  float confidence;
  uint8_t contributingSensors;
  Direction direction;
  bool valid;
};

struct NavigationDecision {
  HazardLevel level;
  Direction direction;
  float distanceCm;
  float closingSpeedCmPerSec;
  float timeToCollisionSec;
  uint8_t speedLimitPercent;
  bool alertEnabled;
};

const char* toString(Direction direction);
const char* toString(HazardLevel level);

}  // namespace nav
