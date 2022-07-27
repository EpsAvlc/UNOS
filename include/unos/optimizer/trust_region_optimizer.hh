#ifndef UNOS_OPTIMIZER_TRUST_REGION_OPTIMIZER_HH__
#define UNOS_OPTIMIZER_TRUST_REGION_OPTIMIZER_HH__

#include "unos/optimizer/optimizer.hh"

namespace unos {
class TrustRegionOptimizer : public Optimizer {
 public:
  void minimize() {
    while (true) {
      
    }
  }

  bool canContinue() {
    // if (iter_num > )
  };

 private:
  int iter_num = 0;
  
};
}  // namespace unos

#endif  // UNOS_OPTIMIZER_TRUST_REGION_OPTIMIZER_HH__
