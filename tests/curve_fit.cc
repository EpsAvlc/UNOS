#include <iostream>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include "unos/unos.hh"

// ax^2 +bx + c = 0
// a = 1, b =2, c = 3
// using LSManifold = unos::Manifold<unos::Vec3>;
class LSCostFunction : public unos::SizedCostFunction<1, 3> {
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

TEST(UNOS, curve_fit) {
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

  Eigen::Vector3d true_solution(1, 2, 3);
  Eigen::Vector3d parameter_vec{param};
  LOG(INFO) << "true   solution: " << true_solution.transpose() << std::endl;
  LOG(INFO) << "solver solution: " << parameter_vec.transpose() << std::endl;
  EXPECT_TRUE(true_solution.isApprox(parameter_vec));
}

// const int kNumObservations = 67;
// // clang-format off
// const double data[] = {
// 0.000000e+00, 1.133898e+00,
// 7.500000e-02, 1.334902e+00,
// 1.500000e-01, 1.213546e+00,
// 2.250000e-01, 1.252016e+00,
// 3.000000e-01, 1.392265e+00,
// 3.750000e-01, 1.314458e+00,
// 4.500000e-01, 1.472541e+00,
// 5.250000e-01, 1.536218e+00,
// 6.000000e-01, 1.355679e+00,
// 6.750000e-01, 1.463566e+00,
// 7.500000e-01, 1.490201e+00,
// 8.250000e-01, 1.658699e+00,
// 9.000000e-01, 1.067574e+00,
// 9.750000e-01, 1.464629e+00,
// 1.050000e+00, 1.402653e+00,
// 1.125000e+00, 1.713141e+00,
// 1.200000e+00, 1.527021e+00,
// 1.275000e+00, 1.702632e+00,
// 1.350000e+00, 1.423899e+00,
// 1.425000e+00, 5.543078e+00, // Outlier point
// 1.500000e+00, 5.664015e+00, // Outlier point
// 1.575000e+00, 1.732484e+00,
// 1.650000e+00, 1.543296e+00,
// 1.725000e+00, 1.959523e+00,
// 1.800000e+00, 1.685132e+00,
// 1.875000e+00, 1.951791e+00,
// 1.950000e+00, 2.095346e+00,
// 2.025000e+00, 2.361460e+00,
// 2.100000e+00, 2.169119e+00,
// 2.175000e+00, 2.061745e+00,
// 2.250000e+00, 2.178641e+00,
// 2.325000e+00, 2.104346e+00,
// 2.400000e+00, 2.584470e+00,
// 2.475000e+00, 1.914158e+00,
// 2.550000e+00, 2.368375e+00,
// 2.625000e+00, 2.686125e+00,
// 2.700000e+00, 2.712395e+00,
// 2.775000e+00, 2.499511e+00,
// 2.850000e+00, 2.558897e+00,
// 2.925000e+00, 2.309154e+00,
// 3.000000e+00, 2.869503e+00,
// 3.075000e+00, 3.116645e+00,
// 3.150000e+00, 3.094907e+00,
// 3.225000e+00, 2.471759e+00,
// 3.300000e+00, 3.017131e+00,
// 3.375000e+00, 3.232381e+00,
// 3.450000e+00, 2.944596e+00,
// 3.525000e+00, 3.385343e+00,
// 3.600000e+00, 3.199826e+00,
// 3.675000e+00, 3.423039e+00,
// 3.750000e+00, 3.621552e+00,
// 3.825000e+00, 3.559255e+00,
// 3.900000e+00, 3.530713e+00,
// 3.975000e+00, 3.561766e+00,
// 4.050000e+00, 3.544574e+00,
// 4.125000e+00, 3.867945e+00,
// 4.200000e+00, 4.049776e+00,
// 4.275000e+00, 3.885601e+00,
// 4.350000e+00, 4.110505e+00,
// 4.425000e+00, 4.345320e+00,
// 4.500000e+00, 4.161241e+00,
// 4.575000e+00, 4.363407e+00,
// 4.650000e+00, 4.161576e+00,
// 4.725000e+00, 4.619728e+00,
// 4.800000e+00, 4.737410e+00,
// 4.875000e+00, 4.727863e+00,
// 4.950000e+00, 4.669206e+00
// };
// clang-format on

// class ExponentialResidual : public unos::SizedCostFunction<2, 1> {
//  public:
//   ExponentialResidual(const double x, const double y) : x_(x), y_(y) {}

//   bool evaluate(double const* const* m, double* residuals,
//                 double** jacobians) const override {
//     if (residuals) {
//       residuals[0] = y_ - exp(m[0][0] * x_ + m[0][1]);
//     }
//     if (jacobians) {

//       jacobians[0][0] = 
//     } 
//     return true;
//   }

//  private:
//   double x_;
//   double y_;
// };

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
