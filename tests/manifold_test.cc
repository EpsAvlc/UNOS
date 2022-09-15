#include <iostream>

#include <gtest/gtest.h>

#include "unos/unos.hh"

TEST(UNOS, manifold) {
  unos::Manifold<unos::SO3, unos::Vec3> state;
  state.setZero();
  unos::Manifold<unos::SO3, unos::Vec3> init_state;
  init_state.setZero();
  state.copyTo(&init_state);

  Eigen::VectorXd perturbance(
      static_cast<int>(unos::Manifold<unos::SO3, unos::Vec3>::DOF));
  for (size_t i = 0; i < state.DOF; ++i) {
    perturbance(i) = 0.01;
  }

  state.oplus(perturbance);

  Eigen::VectorXd pert2 = state.boxminus(&init_state);
  EXPECT_TRUE(pert2.isApprox(perturbance));
}
