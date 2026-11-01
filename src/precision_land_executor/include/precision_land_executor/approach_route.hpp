#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace precision_land_executor {

// A world-frame route contains x, y, z (ENU) triples. Positive z means up.
inline bool validApproachRoute(const std::vector<double>& coordinates) noexcept
{
  if (coordinates.size() < 3 || coordinates.size() % 3 != 0) {
    return false;
  }
  for (std::size_t i = 0; i < coordinates.size(); i += 3) {
    if (!std::isfinite(coordinates[i]) || !std::isfinite(coordinates[i + 1]) ||
        !std::isfinite(coordinates[i + 2]) || coordinates[i + 2] < 2.0) {
      return false;
    }
  }
  return true;
}

}  // namespace precision_land_executor
