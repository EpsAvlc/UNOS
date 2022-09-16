#ifndef UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH
#define UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH

#include "unos/minimizer/minimizer.hh"

namespace unos {
class TrustRegionMinimizer : public Minimizer {
 public:
  void minimize() {
    while (true) {
    }
  }

  bool canContinue(){
      // if (iter_num > )
  };

 private:
  int iter_num = 0;
};
}  // namespace unos

#endif  // UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH
