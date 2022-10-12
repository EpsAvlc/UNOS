#include "unos/minimizer/minimizer.hh"
#include "unos/minimizer/trust_region_minimizer.hh"

namespace unos {

Minimizer::UniquePtr Minimizer::create(const MinimizerType& type) {
  Minimizer::UniquePtr ret;
  switch (type) {
    case TRUST_REGION:
      ret.reset(new TrustRegionMinimizer);
      break;
    default:
      break;
  }
  return ret;
}

}  // namespace unos
