// #include <unistd.h>
// #include <string>

// #include <gtest/gtest.h>
// #include "tests/slam/read_g2o.hh"
// #include "tests/slam/types.hh"
// #include "unos/unos.hh"

// Author: vitus@google.com (Michael Vitus)
// Edit: EpsAvlc

// namespace unos {
// namespace tests {
// void BuildOptimizationProblem(const VectorOfConstraints& constraints,
//                               MapOfPoses* poses, unos::Problem* problem) {
//   CHECK(poses != nullptr);
//   CHECK(problem != nullptr);
//   if (constraints.empty()) {
//     LOG(INFO) << "No constraints, no problem to optimize.";
//     return;
//   }

//   unos::LossFunction* loss_function       = nullptr;
//   // unos::Manifold*     quaternion_manifold = new EigenQuaternionManifold;

//   // for (const auto& constraint : constraints) {
//   //   auto pose_begin_iter = poses->find(constraint.id_begin);
//   //   CHECK(pose_begin_iter != poses->end())
//   //       << "Pose with ID: " << constraint.id_begin << " not found.";
//   //   auto pose_end_iter = poses->find(constraint.id_end);
//   //   CHECK(pose_end_iter != poses->end())
//   //       << "Pose with ID: " << constraint.id_end << " not found.";

//   //   const Eigen::Matrix<double, 6, 6> sqrt_information =
//   //       constraint.information.llt().matrixL();
//   //   // Ceres will take ownership of the pointer.
//   //   unos::CostFunction* cost_function =
//   //   //     PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

//   //   // problem->AddResidualBlock(cost_function, loss_function,
//   //   //                           pose_begin_iter->second.p.data(),
//   //   //                           pose_begin_iter->second.q.coeffs().data(),
//   //   //                           pose_end_iter->second.p.data(),
//   //   //                           pose_end_iter->second.q.coeffs().data());

//   //   // problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
//   //   //                      quaternion_manifold);
//   //   // problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
//   //   //                      quaternion_manifold);
//   // }

//   // The pose graph optimization problem has six DOFs that are not fully
//   // constrained. This is typically referred to as gauge freedom. You can apply
//   // a rigid body transformation to all the nodes and the optimization problem
//   // will still have the exact same cost. The Levenberg-Marquardt algorithm has
//   // internal damping which mitigates this issue, but it is better to properly
//   // constrain the gauge freedom. This can be done by setting one of the poses
//   // as constant so the optimizer cannot change it.
//   auto pose_start_iter = poses->begin();
//   CHECK(pose_start_iter != poses->end()) << "There are no poses.";
//   // problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
//   // problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
// }
// }  // namespace tests
// }  // namespace unos

// TEST(UNOS, pose_graph_3d) {
//   unos::tests::MapOfPoses          poses;
//   unos::tests::VectorOfConstraints constraints;

//   std::string file_path = __FILE__;
//   std::string dir_path  = file_path.substr(0, file_path.rfind("/"));
//   std::string g2o_file  = std::string(dir_path) + "/slam/sphere.g2o";

//   unos::tests::readG2oFile(g2o_file, &poses, &constraints);

//   LOG(INFO) << "Number of poses: " << poses.size() << '\n';
//   LOG(INFO) << "Number of constraints: " << constraints.size() << '\n';

//   unos::Problem proble;
// }
