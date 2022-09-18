#ifndef UNOS_MINIMIZER_MINIMIZER_HH
#define UNOS_MINIMIZER_MINIMIZER_HH

#include <Eigen/Core>
#include <memory>

#include "unos/cost_function/cost_function.hh"
#include "unos/evaluator/evaluator.hh"
#include "unos/manifold/manifold.hh"
#include "unos/problem/program.hh"

namespace unos {
class Minimizer {
 public:
  using Ptr = std::shared_ptr<Minimizer>;
  enum Type { TrustRegion };
  struct Options {
    int            max_iteration_num;
    Evaluator::Ptr evaluator_ptr;
    Program::Ptr   program_ptr;
  };
  virtual void init(const Options& options, double* parameters) = 0;
  // virtual M optimize() = 0;
  virtual void minimize(const Options& options, double* parameters){};
  static void  create();
};
}  // namespace unos

#endif  // UNOS_MINIMIZER_MINIMIZER_HH
