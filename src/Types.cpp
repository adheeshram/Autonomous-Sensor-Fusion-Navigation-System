#include "navigation/Types.h"

namespace nav {

const char* toString(Direction direction) {
  switch (direction) {
    case Direction::None: return "NONE";
    case Direction::Left: return "LEFT";
    case Direction::Center: return "CENTER";
    case Direction::Right: return "RIGHT";
    default: return "UNKNOWN";
  }
}

const char* toString(HazardLevel level) {
  switch (level) {
    case HazardLevel::Clear: return "CLEAR";
    case HazardLevel::Caution: return "CAUTION";
    case HazardLevel::Emergency: return "EMERGENCY";
    default: return "DEGRADED";
  }
}

}  // namespace nav
