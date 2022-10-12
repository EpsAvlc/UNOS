#include <unistd.h>
#include <string>

#include <gtest/gtest.h>
#include "tests/slam/read_g2o.hh"
#include "tests/slam/types.hh"
#include "unos/unos.hh"

// Author: vitus@google.com (Michael Vitus)
// Edit: EpsAvlc

namespace unos {
namespace tests {

class PoseGraph3dErrorTermAnalytic : public unos::SizedCostFunction<6, 3, 4, 3, 4> {
 public:
  PoseGraph3dErrorTermAnalytic(unos::tests::Pose3d         t_ab_measured,
                               Eigen::Matrix<double, 6, 6> sqrt_information)
      : t_ab_measured_(std::move(t_ab_measured)),
        sqrt_information_(std::move(sqrt_information)) {}

  bool evaluate(double const* const* parameters, double* residuals_data,
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

void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses, unos::Problem* problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  unos::LossFunction* loss_function       = nullptr;
  unos::ManifoldBase* quaternion_manifold = new unos::SO3;

  for (const auto& constraint : constraints) {
    auto pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    auto pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    unos::CostFunction* cost_function =
        new PoseGraph3dErrorTermAnalytic(constraint.t_be, sqrt_information);

    problem->addParameterBlock(pose_begin_iter->second.p.data(), 3);
    problem->addParameterBlock(pose_begin_iter->second.q.coeffs().data(), 4,
                               quaternion_manifold);

    problem->addResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->setManifold(pose_begin_iter->second.q.coeffs().data(),
                         quaternion_manifold);
    problem->setManifold(pose_end_iter->second.q.coeffs().data(),
                         quaternion_manifold);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  auto pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->setParameterBlockConstant(pose_start_iter->second.p.data());
  problem->setParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(unos::Problem* problem) {
  CHECK(problem != nullptr);

  problem->optimize();

  return true;
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (const auto& pair : poses) {
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

}  // namespace tests
}  // namespace unos

TEST(UNOS, pose_graph_3d) {
// int main() {
  unos::tests::MapOfPoses          poses;
  unos::tests::VectorOfConstraints constraints;

  std::string file_path = __FILE__;
  std::string dir_path  = file_path.substr(0, file_path.rfind("/"));
  std::string g2o_file  = std::string(dir_path) + "/slam/sphere.g2o";

  unos::tests::readG2oFile(g2o_file, &poses, &constraints);

  LOG(INFO) << "Number of poses: " << poses.size() << '\n';
  LOG(INFO) << "Number of constraints: " << constraints.size() << '\n';

  CHECK(unos::tests::OutputPoses("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  unos::Problem problem;

  unos::tests::BuildOptimizationProblem(constraints, &poses, &problem);

  // CHECK(unos::tests::SolveOptimizationProblem(&problem))
  //     << "The solve was not successful, exiting.";

  // CHECK(unos::tests::OutputPoses("poses_optimized.txt", poses))
  //     << "Error outputting to poses_original.txt";
}
