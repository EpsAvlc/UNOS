#ifndef UNOS_MAINFOLD_MAINFOLD_HH__
#define UNOS_MAINFOLD_MAINFOLD_HH__

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace unos {
template <uint8_t DIM, uint8_t DOF>
class SubManifoldInterface {
 public:
  using StateT = Eigen::Matrix<double, 3, 1>;
  using InputT = Eigen::Matrix<double, 3, 1>;
  virtual void boxplus(const StateT& s) = 0;
  virtual void oplus(const InputT& i) = 0;

 private:
};

class SO3 : public SubManifoldInterface<3, 3> {
 public:
  void boxplus(const StateT& s) override;
  void oplus(const InputT& s) override;

 private:
  Eigen::Quaterniond exp(const Eigen::Matrix<double, 3, 1>& s);
  Eigen::Quaterniond q_;
};

template <uint8_t DIM>
class VecX : public SubManifoldInterface<DIM, DIM> {
 public:
  void boxplus(const StateT& s) override { s_ += s; }
  void oplus(const InputT& s) override { s_ += s; }

 private:
  StateT s_;
};

}  // namespace unos

#endif  // UNOS_MAINFOLD_MAINFOLD_HH__
