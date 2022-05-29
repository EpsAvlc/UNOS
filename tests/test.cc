#include <iostream>

#include <gtest/gtest.h>

#include "unos/unos.hh"

TEST(UNOS, manifold) {
  unos::Manifold state{unos::SO3(), unos::Vec3()};
  state.setZero();
  unos::Manifold init_state{unos::SO3(), unos::Vec3()};
  init_state.setZero();
  state.copyTo(&init_state);

  Eigen::VectorXd perturbance(state.dof());
  for (size_t i = 0; i < state.dof(); ++i) {
    perturbance(i) = 0.01;
  }

  state.oplus(perturbance);

  Eigen::VectorXd pert2 = state.boxminus(&init_state);
  EXPECT_TRUE(pert2.isApprox(perturbance));
}

class OptimizationTest : public ::testing::Test {
 protected:
  void SetUp() override { }

 private:
  //* Powell’s Function, see
  // http://ceres-solver.org/nnls_tutorial.html#powell-s-function
  class f1 : public unos::CostFunction {
   public:
    bool evaluate(const unos::Manifold& m, Eigen::VectorXd* residuals,
                  Eigen::MatrixXd* jacobians) const override {
    
                  }
  };

  unos::Problem problem_;
};
