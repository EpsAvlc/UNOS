#ifndef UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH
#define UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH

#include "unos/minimizer/minimizer.hh"
#include "unos/optimize_strategy/trust_region_strategy.hh"
// #include "unos/problem/program.hh"

namespace unos {
class TrustRegionMinimizer : public Minimizer {
 public:
  enum TerminateCondition {
    ReachMaxIteration,
    Coveraged,
  };

  void init(const Options& options, double* parameters) override final;

  void minimize(const Options& options, double* parameters) override final;

  bool isTerminated(const TerminateCondition* terminate_condition);

 private:
  void iterationZero();

  int                            iter_num = 20;
  Program::Ptr                   program_ptr_;
  TrustRegionStrategy::UniquePtr strategy_ptr_;
  Evaluator::Ptr                 evalutor_ptr_;
  Eigen::VectorXd                x_;
};
}  // namespace unos

#endif  // UNOS_MINIMIZER_TRUST_REGION_MINIMIZER_HH
