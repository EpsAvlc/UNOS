#include <iostream>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "unos/unos.hh"

// int main() {}
TEST(UNOS, manifold) {
  double                         T0[] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q0(T0);
  Eigen::Map<Eigen::Vector3d>    t0(T0 + 4);
  double T1[] = {sin(M_PI / 6), 0, 0, cos(M_PI / 6), 10, 20, 30};
  Eigen::Map<Eigen::Quaterniond> q1(T1);
  Eigen::Map<Eigen::Vector3d>    t1(T1 + 4);
  double                         T0_plus_T1[7];
  Eigen::Map<Eigen::Quaterniond> q0_plus_q1(T0_plus_T1);
  Eigen::Map<Eigen::Vector3d>    t0_plus_t1(T0_plus_T1 + 4);
  Eigen::Quaterniond             delta_q(1, 0.001, 0.001, 0.001);
  delta_q.normalize();
  Eigen::Vector3d delta_t(0.001, 0.001, 0.001);
  double          delta_T[7];
  for (int i = 0; i < 4; ++i) {
    delta_T[i] = delta_q.coeffs()[i];
  }
  for (int i = 0; i < 3; ++i) {
    delta_T[i + 4]       = delta_t[i];
  }

  // VEC3 test
  unos::Vec3 vec3;
  vec3.boxplus(t0.data(), t1.data(), t0_plus_t1.data());
  LOG(INFO) << "t0 + t1: " << (t0 + t1);
  LOG(INFO) << "t0_plus_t1: " << (t0_plus_t1);
  EXPECT_TRUE(t0_plus_t1.isApprox(t0 + t1));
  Eigen::Matrix3d t1_jacobian;
  vec3.oplusJacobian(t1.data(), t1_jacobian.data());
  EXPECT_TRUE(t1_jacobian.isApprox(Eigen::Matrix3d::Identity()));

  // SO3 test
  unos::SO3 so3;
  so3.boxplus(q0.coeffs().data(), q1.coeffs().data(),
              q0_plus_q1.coeffs().data());
  LOG(INFO) << "q0 * q1: " << (q0 * q1).coeffs();
  LOG(INFO) << "q0_plus_q1: " << (q0_plus_q1).coeffs();
  EXPECT_TRUE(q0_plus_q1.isApprox(q0 * q1));
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> q1_jacobian;
  so3.oplusJacobian(q1.coeffs().data(), q1_jacobian.data());
  Eigen::Vector4d delta_q1_minus_q1 = (delta_q * q1).coeffs() - q1.coeffs();
  LOG(INFO) << "q1: " << q1.coeffs().transpose();
  LOG(INFO) << "q1_jacobian: " << std::endl << q1_jacobian;
  LOG(INFO) << "delta_q1_minus_q1: " << delta_q1_minus_q1.transpose();
  LOG(INFO) << "q1_jacobiaon * delta_q: "
            << (q1_jacobian * delta_q.vec()).transpose();
  EXPECT_TRUE(delta_q1_minus_q1.isApprox(q1_jacobian * delta_q.vec(), 0.001));

  // SE3 test
  using SE3 = unos::Manifold<unos::SO3, unos::Vec3>;
  SE3 se3;
  se3.boxplus(T0, T1, T0_plus_T1);
  LOG(INFO) << "q0 * q1: " << (q0 * q1).coeffs();
  LOG(INFO) << "q0_plus_q1: " << (q0_plus_q1).coeffs();
  EXPECT_TRUE(q0_plus_q1.isApprox(q0 * q1));
  LOG(INFO) << "t0 + t1: " << (t0 + t1);
  LOG(INFO) << "t0_plus_t1: " << (t0_plus_t1);
  EXPECT_TRUE(t0_plus_t1.isApprox(t0 + t1));
  double T0_plus_delta_T[7];
  se3.boxplus(T0, delta_T, T0_plus_delta_T);
  Eigen::Map<Eigen::Quaterniond> q0_plus_delta_q(T0_plus_delta_T);
  Eigen::Map<Eigen::Vector3d>    t0_plus_delta_t(T0_plus_delta_T + 4);
  EXPECT_TRUE(q0_plus_delta_q.isApprox(delta_q * q0));
  EXPECT_TRUE(t0_plus_delta_t.isApprox(t0 + delta_t));
  Eigen::Matrix<double, SE3::DIM, SE3::DOF, Eigen::RowMajor> T1_jacobian;
  se3.oplusJacobian(T1, T1_jacobian.data());
  LOG(INFO) << "T1_jacobian: " << std::endl << T1_jacobian;
  EXPECT_TRUE(T1_jacobian.block(0, 0, 4, 3).isApprox(q1_jacobian, 0.001));
  EXPECT_TRUE(T1_jacobian.block(4, 3, 3, 3).isApprox(t1_jacobian));
}
