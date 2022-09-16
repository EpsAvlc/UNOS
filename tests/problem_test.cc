#include <iostream>

#include <gtest/gtest.h>

#include "unos/unos.hh"

// ax^2 +bx + c = 0
// a = 1, b =2, c = 3
// using LSManifold = unos::Manifold<unos::Vec3>;
class LSCostFunction : public unos::AnalyticCostFunction<3, 1> {
 public:
  LSCostFunction(double x, double y) : x_(x), y_(y) {}
  bool evaluate(double const* const* m, double* residuals,
                double** jacobians) const override {
    double a     = m[0][0];
    double b     = m[0][1];
    double c     = m[0][2];
    residuals[0] = x_ * x_ * a + x_ * b + c - y_;
    if (jacobians) {
      jacobians[0][0] = x_ * x_;
      jacobians[0][1] = x_;
      jacobians[0][2] = 1;
    }
    return true;
  }

 private:
  double x_;
  double y_;
};
TEST(UNOS, least_square) {
  double        param[] = {0, 0, 0};
  unos::Problem problem;
  problem.addParameterBlock(param, 3);

  unos::HuberLossFunction* huber_lf   = new unos::HuberLossFunction(0.1);
  LSCostFunction*          cost_func1 = new LSCostFunction(1, 6);
  problem.addResidualBlock(cost_func1, huber_lf, param);
  LSCostFunction* cost_func2 = new LSCostFunction(2, 11);
  problem.addResidualBlock(cost_func2, huber_lf, param);
  LSCostFunction* cost_func3 = new LSCostFunction(0, 3);
  problem.addResidualBlock(cost_func3, huber_lf, param);

  problem.optimize();

  // std::cout << "hello." << std::endl;
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
