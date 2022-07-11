#ifndef UNOS_LOSS_FUNCTION_H__
#define UNOS_LOSS_FUNCTION_H__

namespace unos {
class LossFunction {
 public:
  virtual void Evaluate(double sq_norm, double out[3]) const = 0;
};
};  // namespace unos

#endif  // LOSE_FUNCTION_HH
