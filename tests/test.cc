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

// class OptimizationTest : public ::testing::Test {
//  protected:
//   void SetUp() override {}

//  private:
//   //* Powell’s Function, see
//   // http://ceres-solver.org/nnls_tutorial.html#powell-s-function
//   class f1 : public unos::CostFunction {
//    public:
//     bool evaluate(const unos::Manifold& m, Eigen::VectorXd* residuals,
//                   Eigen::MatrixXd* jacobians) const override {
//       residuals->resize(1);
//       (*residuals)(0) = m.coeffs()[0] + 10 * m.coeffs()[1];
//     }
//   };

//   class f2 : public unos::CostFunction {
//    public:
//     bool evaluate(const unos::Manifold& m, Eigen::VectorXd* residuals,
//                   Eigen::MatrixXd* jacobians) const override {
//       residuals->resize(1);
//       (*residuals)(0) = std::sqrt(5) * (m.coeffs()[2] + 10 * m.coeffs()[3]);
//     }
//   };

//   class f3 : public unos::CostFunction {
//    public:
//     bool evaluate(const unos::Manifold& m, Eigen::VectorXd* residuals,
//                   Eigen::MatrixXd* jacobians) const override {
//       residuals->resize(1);
//       (*residuals)(0) = (m.coeffs()[1] - 2 * m.coeffs()[2]) *
//                         (m.coeffs()[1] - 2 * m.coeffs()[2]);
//     }
//   };

//   unos::Problem problem_;
// };

using LQManifold = unos::Manifold<unos::Vec3>;
class LSQCostFunction : public unos::CostFunction<LQManifold> {
 public:
  bool evaluate(const LQManifold& m, Eigen::VectorXd* residuals,
                Eigen::MatrixXd* jacobians) const {}
};
TEST(UNOS, least_square) {
  LQManifold::Ptr abc(new LQManifold);
  unos::Problem<LQManifold> problem(abc);
  // problem.addCostFunctions
}
