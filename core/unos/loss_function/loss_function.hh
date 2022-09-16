#ifndef UNOS_LOSS_FUNCTION_LOSS_FUNCTION_HH
#define UNOS_LOSS_FUNCTION_LOSS_FUNCTION_HH

namespace unos {
class LossFunction {
 public:
  virtual void Evaluate(double sq_norm, double out[3]) const = 0;
};
};  // namespace unos

#endif // UNOS_LOSS_FUNCTION_LOSS_FUNCTION_HH
