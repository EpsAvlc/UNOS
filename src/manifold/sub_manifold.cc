#include "unos/manifold/sub_manifold.hh"

void unos::SO3::boxplus(const StateT& s) { q_ = q_ * exp(s); }

void unos::SO3::oplus(const InputT& s) { q_ = q_ * exp(s); }

Eigen::Quaterniond unos::SO3::exp(const Eigen::Matrix<double, 3, 1>& s) {
  Eigen::Quaterniond ret;
  double norm = s.norm();
  ret.vec() = sin(norm / 2) / norm * s;
  ret.w() = cos(norm / 2);
}
