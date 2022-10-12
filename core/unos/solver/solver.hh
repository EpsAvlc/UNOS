#ifndef UNOS_SOLVER_SOLVER_HH
#define UNOS_SOLVER_SOLVER_HH

#include "unos/minimizer/minimizer.hh"
#include "unos/problem/problem.hh"
#include "unos/types/types.h"

namespace unos {
class Solver {
 public:
  struct Options {
    MinimizerType    minimizer_type     = MinimizerType::TRUST_REGION;
    LinearSolverType linear_solver_type = LinearSolverType::DENSE_QR;
  };

  struct Summary {
    std::string briefReport() const;

    std::string fullReport() const;

    bool isSolutionUsable() const;

    MinimizerType minimizer_type = MinimizerType::TRUST_REGION;

    TerminationType termination_type = TerminationType::FAILURE;
  };

  virtual void solve(const Options& options, Problem* problem,
                     Solver::Summary* Summary);

 private:
};
}  // namespace unos

#endif  // UNOS_SOLVER_SOLVER_HH
