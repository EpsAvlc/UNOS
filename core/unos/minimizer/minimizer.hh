#ifndef UNOS_MINIMIZER_MINIMIZER_HH
#define UNOS_MINIMIZER_MINIMIZER_HH

#include <Eigen/Core>
#include <memory>

#include "unos/cost_function/cost_function.hh"
#include "unos/evaluator/evaluator.hh"
#include "unos/types/types.h"

namespace unos {
class Minimizer {
 public:
  using Ptr       = std::shared_ptr<Minimizer>;
  using UniquePtr = std::unique_ptr<Minimizer>;
  struct Options {
    int            max_iteration_num;
    Evaluator::Ptr evaluator_ptr;
    Program::Ptr   program_ptr;
    // typename TrustRegionStrategy::Type strategy_type =
    //     TrustRegionStrategy::LevenbergMarquart;
  };
  virtual void init(const Options& options, double* parameters) = 0;
  virtual void minimize(const Options& options, double* parameters){};

  static UniquePtr create(const MinimizerType& type);
};
}  // namespace unos

#endif  // UNOS_MINIMIZER_MINIMIZER_HH
