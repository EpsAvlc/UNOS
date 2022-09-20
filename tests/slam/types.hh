#ifndef UNOS_SLAM_TYPES_HH
#define UNOS_SLAM_TYPES_HH

// Author: vitus@google.com (Michael Vitus)
// Edit: EpsAvlc

#include <functional>
#include <istream>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace unos {
namespace tests {

struct Pose3d {
  Eigen::Vector3d    p;
  Eigen::Quaterniond q;

  // The name of the data type in the g2o file format.
  static std::string name() { return "VERTEX_SE3:QUAT"; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::istream& operator>>(std::istream& input, Pose3d& pose) {
  input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >>
      pose.q.z() >> pose.q.w();
  // Normalize the quaternion to account for precision loss due to
  // serialization.
  pose.q.normalize();
  return input;
}

using MapOfPoses =
    std::map<int, Pose3d, std::less<int>,
             Eigen::aligned_allocator<std::pair<const int, Pose3d>>>;

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint3d {
  int id_begin;
  int id_end;

  // The transformation that represents the pose of the end frame E w.r.t. the
  // begin frame B. In other words, it transforms a vector in the E frame to
  // the B frame.
  Pose3d t_be;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, z, delta orientation.
  Eigen::Matrix<double, 6, 6> information;

  // The name of the data type in the g2o file format.
  static std::string name() { return "EDGE_SE3:QUAT"; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::istream& operator>>(std::istream& input, Constraint3d& constraint) {
  Pose3d& t_be = constraint.t_be;
  input >> constraint.id_begin >> constraint.id_end >> t_be;

  for (int i = 0; i < 6 && input.good(); ++i) {
    for (int j = i; j < 6 && input.good(); ++j) {
      input >> constraint.information(i, j);
      if (i != j) {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }
  return input;
}

using VectorOfConstraints =
    std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>;

}  // namespace examples
}  // namespace unos

#endif  // UNOS_SLAM_TYPES_HH
