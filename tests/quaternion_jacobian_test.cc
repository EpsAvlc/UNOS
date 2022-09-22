#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

TEST(UNOS, quaternion_jacobian_test) {
  constexpr double   theta = 30 * M_PI / 180;
  Eigen::Vector3d    axis(0, 1, 0);
  Eigen::Quaterniond q(cos(theta), axis.x() * sin(theta), axis.y() * sin(theta),
                       axis.z() * sin(theta));
  q.normalize();
  Eigen::Vector3d p(20, 10, -10);
  Eigen::Quaterniond v_p(0, p.x(), p.y(), p.z());

  Eigen::Vector3d res1 = q * p;
  Eigen::Vector3d res2 = (q * v_p * q.conjugate()).vec();
  EXPECT_TRUE(res1.isApprox(res2));
  // Eigen::Quaterniond
}
