#ifndef UNOS_LOSS_FUNCTION_HUBER_LOSS_FUNCTION_HH
#define UNOS_LOSS_FUNCTION_HUBER_LOSS_FUNCTION_HH

#include <cmath>
#include <numeric>
#include "unos/loss_function/loss_function.hh"

namespace unos {
class HuberLossFunction : public LossFunction {
 public:
  HuberLossFunction(const double rho) : rho_(rho), rho_square_(rho * rho) {}
  void Evaluate(double sq_norm, double out[3]) const override {
    if (sq_norm < rho_square_) {
      out[0] = sq_norm;
      out[1] = 1.0;
      out[2] = 0;
    } else {
      double norm = std::sqrt(sq_norm);
      out[0] = 2 * rho_ * norm - rho_square_;
      out[1] = std::max(std::numeric_limits<double>::min(), rho_ / norm);
      out[2] = -0.5 * out[1] / sq_norm;
    }
  }

 private:
  double rho_;
  double rho_square_;
};
};  // namespace unos

#endif // UNOS_LOSS_FUNCTION_HUBER_LOSS_FUNCTION_HH
