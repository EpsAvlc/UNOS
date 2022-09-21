#include <filesystem>

#include "unos/unos.hh"
#include "tests/slam/types.hh"
#include "tests/slam/read_g2o.hh"
#include "gtest/gtest.h"

// Author: vitus@google.com (Michael Vitus)
// Edit: EpsAvlc

TEST(UNOS, pose_graph_3d) {
  unos::tests::MapOfPoses poses;
  unos::tests::VectorOfConstraints constraints;
  char current_path[PATH_MAX];
  getcwd(current_path, sizeof(current_path));
  std::string g2o_file = current_path + "../";
  // unos::tests::readG2oFile()
}
