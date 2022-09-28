#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "tests/slam/types.hh"

namespace unos {
namespace tests {

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraph3dErrorTerm {
 public:
  PoseGraph3dErrorTerm(unos::tests::Pose3d         t_ab_measured,
                       Eigen::Matrix<double, 6, 6> sqrt_information)
      : t_ab_measured_(std::move(t_ab_measured)),
        sqrt_information_(std::move(sqrt_information)) {}

  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                  const T* const p_b_ptr, const T* const q_b_ptr,
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>>   q_a(q_a_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>>   q_b(q_b_ptr);

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse    = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - t_ab_measured_.p.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const Pose3d&                      t_ab_measured,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
        new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The measurement for the position of B relative to A in the A frame.
  const Pose3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class PoseGraph3dErrorTermAnalytic
    : public ceres::SizedCostFunction<6, 3, 4, 3, 4> {
 public:
  PoseGraph3dErrorTermAnalytic(unos::tests::Pose3d         t_ab_measured,
                               Eigen::Matrix<double, 6, 6> sqrt_information)
      : t_ab_measured_(std::move(t_ab_measured)),
        sqrt_information_(std::move(sqrt_information)) {}

  bool Evaluate(double const* const* parameters, double* residuals_data,
                double** jacobians) const override {
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> p_a(parameters[0]);
    Eigen::Map<const Eigen::Quaternion<double>>   q_a(parameters[1]);

    Eigen::Map<const Eigen::Matrix<double, 3, 1>> p_b(parameters[2]);
    Eigen::Map<const Eigen::Quaternion<double>>   q_b(parameters[3]);

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<double> q_a_inverse    = q_a.conjugate();
    Eigen::Quaternion<double> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<double, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<double> delta_q =
        t_ab_measured_.q.template cast<double>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals(residuals_data);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - t_ab_measured_.p.template cast<double>();
    residuals.template block<3, 1>(3, 0) = double(2.0) * delta_q.vec();
    residuals.applyOnTheLeft(sqrt_information_.template cast<double>());

    LOG(INFO) << "Prepare to evaluate jacobian";
    if (!jacobians) {
      return true;
    }

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jaco_pa(
          jacobians[0]);
      jaco_pa.block(0, 0, 3, 3) = -q_a.conjugate().toRotationMatrix();
      jaco_pa.block(3, 0, 3, 3).setZero();

      jaco_pa.applyOnTheLeft(sqrt_information_);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jaco_qa(
          jacobians[1]);
      Eigen::Vector3d pba = p_b - p_a;
      jaco_qa.block(0, 0, 3, 4) =
          rotationJaco(q_a.conjugate(), pba) * inverseJaco();
      jaco_qa.block(3, 0, 3, 4) =
          2 * quaternionToVec() *
          leftRotationMatrix(t_ab_measured_.q * q_b.conjugate());

      jaco_qa.applyOnTheLeft(sqrt_information_);
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jaco_pb(
          jacobians[2]);
      jaco_pb.block(0, 0, 3, 3) = q_a.conjugate().toRotationMatrix();
      jaco_pb.block(3, 0, 3, 3).setZero();

      jaco_pb.applyOnTheLeft(sqrt_information_);
    }

    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jaco_qb(
          jacobians[3]);
      jaco_qb.block(0, 0, 3, 4).setZero();
      jaco_qb.block(3, 0, 3, 4) = 2 * quaternionToVec() *
                                  leftRotationMatrix(t_ab_measured_.q) *
                                  rightRotationMatrix(q_a) * inverseJaco();

      jaco_qb.applyOnTheLeft(sqrt_information_);
    }

    LOG(INFO) << "Finish to evaluate jacobian";
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Matrix<double, 3, 4> quaternionToVec() const {
    Eigen::Matrix<double, 3, 4> ret;
    ret.setZero();
    ret.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    return ret;
  }

  Eigen::Matrix<double, 4, 4> leftRotationMatrix(
      const Eigen::Quaterniond& q) const {
    Eigen::Matrix<double, 4, 4> ret;
    ret(0, 0) = q.w();
    ret(0, 1) = -q.z();
    ret(0, 2) = q.y();
    ret(0, 3) = q.x();
    ret(1, 0) = q.z();
    ret(1, 1) = q.w();
    ret(1, 2) = -q.x();
    ret(1, 3) = q.y();
    ret(2, 0) = -q.y();
    ret(2, 1) = q.x();
    ret(2, 2) = q.w();
    ret(2, 3) = q.z();
    ret(3, 0) = -q.x();
    ret(3, 1) = -q.y();
    ret(3, 2) = -q.z();
    ret(3, 3) = q.w();

    return ret;
  }

  Eigen::Matrix<double, 4, 4> rightRotationMatrix(
      const Eigen::Quaterniond& q) const {
    Eigen::Matrix<double, 4, 4> ret;
    ret(0, 0) = q.w();
    ret(0, 1) = q.z();
    ret(0, 2) = -q.y();
    ret(0, 3) = q.x();
    ret(1, 0) = -q.z();
    ret(1, 1) = q.w();
    ret(1, 2) = q.x();
    ret(1, 3) = q.y();
    ret(2, 0) = q.y();
    ret(2, 1) = -q.x();
    ret(2, 2) = q.w();
    ret(2, 3) = q.z();
    ret(3, 0) = -q.x();
    ret(3, 1) = -q.y();
    ret(3, 2) = -q.z();
    ret(3, 3) = q.w();

    return ret;
  }

  Eigen::Matrix<double, 3, 4> rotationJaco(const Eigen::Quaterniond& q,
                                           const Eigen::Vector3d&    p) const {
    Eigen::Matrix<double, 3, 4> ret;
    ret.setZero();
    ret(0, 0) = q.y() * p.y() + q.z() * p.z();
    ret(0, 1) = -2 * q.y() * p.x() + q.x() * p.y() + q.w() * p.z();
    ret(0, 2) =
        -2 * q.z() * p.x() - q.w() * p.y() - q.z() * p.y() + q.y() * p.z();
    ret(1, 0) = q.y() * p.x() - 2 * q.x() * p.y() - q.w() * p.z();
    ret(1, 1) = q.x() * p.x() + q.z() * p.z();
    ret(1, 2) = q.w() * p.x() - 2 * q.z() * p.y() + q.y() * p.z();
    ret(1, 3) = q.z() * p.x() - q.x() * p.z();
    ret(2, 0) = q.z() * p.x() + q.w() * p.y() - 2 * q.x() * p.z();
    ret(2, 1) = -q.w() * p.x() + q.z() * p.y() - 2 * q.y() * p.z();
    ret(2, 2) = q.x() * p.x() + q.y() * p.y();
    ret(2, 3) = -q.y() * p.x() + q.x() * p.y();

    ret *= 2;
    return ret;
  }

  Eigen::Matrix<double, 4, 4> inverseJaco() const {
    Eigen::Matrix<double, 4, 4> ret;
    ret.setIdentity();
    ret.block(0, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
    return ret;
  }
  // The measurement for the position of B relative to A in the A frame.
  const Pose3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

}  // namespace tests
}  // namespace unos

TEST(UNOS, pose_graph_3d_error_term) {
  // int main() {
  Eigen::Vector3d    p_a(0, 0, 0);
  Eigen::Quaterniond q_a(1, 0, 0, 0);
  Eigen::Vector3d    p_b(1, 1, 1);
  Eigen::Quaterniond q_b(cos(30 * M_PI / 180), sin(30 * M_PI / 180), 0, 0);

  Eigen::Matrix<double, 6, 6> sqrt_info;
  sqrt_info.setIdentity();

  unos::tests::Pose3d t_ab_measured;
  t_ab_measured.p = q_a.inverse() * (p_b - p_a);
  t_ab_measured.q = q_a.inverse() * q_b;

  unos::tests::PoseGraph3dErrorTerm automatic_error_term(t_ab_measured,
                                                         sqrt_info);

  ceres::Jet<double, 14> p_a_jet[3], p_b_jet[3];
  for (int i = 0; i < 3; ++i) {
    p_a_jet[i].a = p_a[i];
    p_a_jet[i].v.setZero();
    p_a_jet[i].v[i] = 1.;

    p_b_jet[i].a = p_b[i];
    p_b_jet[i].v.setZero();
    p_b_jet[i].v[i + 7] = 1.;
  }

  ceres::Jet<double, 14> q_a_jet[4], q_b_jet[4];
  for (int i = 0; i < 4; ++i) {
    q_a_jet[i].a = q_a.coeffs()[i];
    q_a_jet[i].v.setZero();
    q_a_jet[i].v[i + 3] = 1.;

    q_b_jet[i].a = q_b.coeffs()[i];
    q_b_jet[i].v.setZero();
    q_b_jet[i].v[i + 10] = 1.;
  }

  ceres::Jet<double, 14> residuals[6];
  automatic_error_term(p_a_jet, q_a_jet, p_b_jet, q_b_jet, residuals);

  Eigen::Matrix<double, 6, 14> jacobian_auto;

  for (size_t i = 0; i < 6; ++i) {
    jacobian_auto.row(i) = residuals[i].v.transpose();
  }
  std::cout << "automatic jacobian: " << std::endl << jacobian_auto << std::endl;

  unos::tests::PoseGraph3dErrorTermAnalytic analytic_error_term(t_ab_measured,
                                                                sqrt_info);
  double* parameters[4] = {p_a.data(), q_a.coeffs().data(), p_b.data(),
                           q_b.coeffs().data()};
  double  residuals_al[6]  = {0};
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> p_a_jaco, p_b_jaco;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> q_a_jaco, q_b_jaco;
  double* jacobians_al[4] = {p_a_jaco.data(), q_a_jaco.data(), p_b_jaco.data(),
                          q_b_jaco.data()};

  Eigen::Matrix<double, 6, 14> jacobian_analy;
  analytic_error_term.Evaluate(parameters, residuals_al, jacobians_al);
  jacobian_analy.block(0, 0, 6, 3) = p_a_jaco;
  jacobian_analy.block(0, 3, 6, 4) = q_a_jaco;
  jacobian_analy.block(0, 7, 6, 3) = p_b_jaco;
  jacobian_analy.block(0, 10, 6, 4) = q_b_jaco;

  std::cout << "analytic jacobian: " << std::endl << jacobian_analy << std::endl;
  EXPECT_TRUE(jacobian_auto.isApprox(jacobian_analy));
}
