#include "unos/optimize_strategy/trust_region_strategy.hh"
#include "unos/optimize_strategy/levenberg_marquardt_strategy.hh"

namespace unos {

TrustRegionStrategy::UniquePtr TrustRegionStrategy::create(const Type& type) {
  TrustRegionStrategy::UniquePtr ret;
  switch (type) {
    case LevenbergMarquart:
      return std::make_unique<LevenbergMarquardtStratege>();
      break;

    default:
      return ret;
      break;
  }
}

}  // namespace unos
