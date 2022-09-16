#ifndef UNOS_MINIMIZER_MINIMIZER_HH
#define UNOS_MINIMIZER_MINIMIZER_HH

#include <Eigen/Core>
#include <memory>

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"

namespace unos {
class Minimizer {
 public:
  using Ptr = std::shared_ptr<Minimizer>;
  struct Options {
    int max_iteration_num;
  };
  virtual void init(const Options& options) = 0;
  // virtual M optimize() = 0;
  virtual void optimize(){};
};
}  // namespace unos

#endif // UNOS_MINIMIZER_MINIMIZER_HH
