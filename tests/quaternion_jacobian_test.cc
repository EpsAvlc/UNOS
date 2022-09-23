#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 4> rotationJaco(const Eigen::Quaterniond& q,
                                         const ::Eigen::Vector3d&  p) {
  Eigen::Matrix<double, 3, 4> ret;
  ret.setZero();
  ret(0, 0) = q.y() * p.y() + q.z() * p.z();
  ret(0, 1) = -2 * q.y() * p.x() + q.x() * p.y() + q.w() * p.z();
  ret(0, 2) =
      -2 * q.z() * p.x() - q.w() * p.y() - q.z() * p.y() + q.y() * p.z();
  ret(1, 0) = q.y() * p.x() - 2 * q.x() * p.y() - q.w() * p.z();
  ret(1, 1) = q.x() * p.x() + q.z() * p.z();
  ret(1, 2) = q.w() * p.x() - 2 * q.z() * p.y() + q.y() *p.z();
  ret(1, 3) = q.z() * p.x() - q.x() * p.z();
  ret(2, 0) = q.z() * p.x() + q.w() * p.y() - 2 * q.x() * p.z();
  ret(2, 1) = -q.w() * p.x() + q.z() * p.y() - 2 * q.y() * p.z();
  ret(2, 2) = q.x() * p.x() + q.y() * p.y();
  ret(2, 3) = -q.y() * p.x() + q.x() * p.y();

  ret *= 2;
  return ret;
}

TEST(UNOS, quaternion_jacobian_test) {
  constexpr double   theta = 30 * M_PI / 180;
  Eigen::Vector3d    axis(0, 1, 0);
  Eigen::Quaterniond q(cos(theta), axis.x() * sin(theta), axis.y() * sin(theta),
                       axis.z() * sin(theta));
  q.normalize();
  Eigen::Vector3d    p(20, 10, -10);
  Eigen::Quaterniond v_p(0, p.x(), p.y(), p.z());

  Eigen::Vector3d res1 = q * p;
  Eigen::Vector3d res2 = (q * v_p * q.conjugate()).vec();
  EXPECT_TRUE(res1.isApprox(res2));

  Eigen::Vector4d delta_q(0.0001, 0.0001, 0.0001, 0.0001);
  Eigen::Quaterniond plus_q;
  plus_q.coeffs() = q.coeffs() + delta_q;
  LOG(INFO) << "plus q: " << plus_q.coeffs().transpose() << std::endl;

  Eigen::Matrix<double, 3, 4> jaco    = rotationJaco(q, p);
  Eigen::Vector3d             delta_y = plus_q * p - res1;
  LOG(INFO) << "deri: " << delta_y.transpose() << std::endl;
  Eigen::Vector3d             approx_delta_y = jaco * delta_q;
  LOG(INFO) << "jaco * delta: " << approx_delta_y.transpose() << std::endl;

  EXPECT_TRUE(delta_y.isApprox(approx_delta_y, 0.001));
  // Eigen::Quaterniond
}
