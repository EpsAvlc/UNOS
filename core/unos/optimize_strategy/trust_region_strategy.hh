#ifndef UNOS_OPTIMIZE_STRATEGY_OPTIMIZE_STRATEGY_HH
#define UNOS_OPTIMIZE_STRATEGY_OPTIMIZE_STRATEGY_HH

#include <memory>
#include "unos/sparse_matrix/sparse_matrix.hh"

namespace unos {
class TrustRegionStrategy {
 public:
  using UniquePtr = std::unique_ptr<TrustRegionStrategy>;
  enum Type { LevenbergMarquart };

  struct Summary {
    bool success;
  };

  virtual void    init(const SparseMatrix::UniquePtr& jacobian) = 0;
  virtual Summary computeStep(const SparseMatrix::UniquePtr& jacobian,
                              const double* residuals, double* step) = 0;

  static TrustRegionStrategy::UniquePtr create(const Type& type);

 private:
};
}  // namespace unos

#endif  // UNOS_OPTIMIZE_STRATEGY_OPTIMIZE_STRATEGY_HH
