#ifndef UNOS_MANIFOLD_SO3_HH
#define UNOS_MANIFOLD_SO3_HH

#include <cstdarg>
#include "unos/manifold/sub_manifold.hh"

namespace unos {
class SO3 : public SubManifold {
 public:
  enum : int { DIM = 4, DOF = 3 };
  using Ptr = std::shared_ptr<SO3>;

  void boxplus(double const* const x, double const* const y,
               double* x_plus_y) const override {
    Eigen::Map<const Eigen::Quaterniond> q_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_y(y);
    Eigen::Map<Eigen::Quaterniond>       q_x_plus_q_y(x_plus_y);
    q_x_plus_q_y = q_x * q_y;
  }

  void oplus(double const* const x, double const* const delta,
             double* x_plus_delta) const override {
    const double norm_delta = std::sqrt(
        delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);

    if (norm_delta == 0.0) {
      for (int i = 0; i < 4; ++i) {
        x_plus_delta[i] = x[i];
      }
      return;
    }

    const double       sin_delta_by_delta = (std::sin(norm_delta) / norm_delta);
    Eigen::Quaterniond q_delta(
        std::cos(norm_delta), sin_delta_by_delta * delta[0],
        sin_delta_by_delta * delta[1], sin_delta_by_delta * delta[2]);

    Eigen::Map<const Eigen::Quaterniond> q_x(x);
    Eigen::Map<Eigen::Quaterniond>       q_x_plus_delta(x_plus_delta);
    // Note: delta is added on the left side.
    q_x_plus_delta = q_delta * q_x;
  }

  void oplusJacobian(double const* const x, double* jacobian) const override {
    Eigen::Map<const Eigen::Quaterniond>                     q_x(x);
    Eigen::Map<Eigen::Matrix<double, DIM, DOF, Eigen::RowMajor>> jaco_mat(jacobian);
    // clang-format off
    jaco_mat <<  q_x.w(),  q_x.z(), -q_x.y(),
                -q_x.z(),  q_x.w(),  q_x.x(),
                 q_x.y(), -q_x.x(),  q_x.w(),
                -q_x.x(), -q_x.y(), -q_x.z();
    // clang-format on
  }

  int dim() const { return DIM; };
  int dof() const { return DOF; };

  std::string typeID() const override { return "SO3"; };

 private:
};

REGISTER_UNOS(SubManifold, SO3, "SO3");

}  // namespace unos

#endif  // UNOS_MANIFOLD_SO3_HH
