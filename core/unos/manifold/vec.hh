#ifndef UNOS_MANIFOLD_VEC_HH
#define UNOS_MANIFOLD_VEC_HH

#include <cstdarg>
#include "unos/manifold/manifold_base.hh"

namespace unos {
template <int N>
class Vec : public ManifoldBase {
 public:
  using Ptr = std::shared_ptr<Vec<N>>;
  enum : int { DIM = N, DOF = N };
  void boxplus(double const* const x, double const* const y,
               double* x_plus_y) const override {
    for (size_t i = 0; i < DIM; ++i) {
      x_plus_y[i] = x[i] + y[i];
    }
  }

  void oplus(double const* const x, double const* const delta_x,
             double* x_plus_delta_x) const override {
    boxplus(x, delta_x, x_plus_delta_x);
  }

  void oplusJacobian(double const* const x, double* jacobian) const override {
    Eigen::Map<const Eigen::Quaterniond>                         q_x(x);
    Eigen::Map<Eigen::Matrix<double, DIM, DOF, Eigen::RowMajor>> jaco_mat(
        jacobian);
    jaco_mat.setIdentity();
  }

  std::string typeID() const override { return "Vec" + std::to_string(N); };

  int dof() const override { return DOF; }

  int dim() const override { return DIM; }
};

using Vec1 = Vec<1>;
using Vec2 = Vec<2>;
using Vec3 = Vec<3>;
REGISTER_UNOS(ManifoldBase, Vec1, "Vec1");
REGISTER_UNOS(ManifoldBase, Vec2, "Vec2");
REGISTER_UNOS(ManifoldBase, Vec3, "Vec3");
}  // namespace unos

#endif  // UNOS_MANIFOLD_VEC_HH
