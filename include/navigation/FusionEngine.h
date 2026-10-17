#pragma once

#include "navigation/Types.h"

namespace nav {

class FusionEngine {
 public:
  FusedEstimate fuse(const FilteredMeasurement* measurements,
                     uint8_t count,
                     uint32_t nowMs) const;

 private:
  struct SectorEstimate {
    float distanceCm;
    float weight;
    uint8_t contributors;
    bool valid;
  };

  SectorEstimate fuseSector(const FilteredMeasurement* measurements,
                            uint8_t count,
                            Sector sector,
                            uint32_t nowMs) const;
};

}  // namespace nav
