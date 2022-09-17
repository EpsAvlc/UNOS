cmake_minimum_required(VERSION 3.0.0)
project(UNOS VERSION 0.1.0)

find_package(GTest REQUIRED)
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "DEBUG")

file(GLOB_RECURSE ${PROJECT_NAME}_CORE_FILES "${CMAKE_CURRENT_LIST_DIR}/core/*.cc")
message("UNOS_CORE FILES: ")
message("${CMAKE_CURRENT_LIST_DIR}/core")
message("${UNOS_CORE_FILES}")

include_directories(
  /usr/include/eigen3
  ${CMAKE_CURRENT_LIST_DIR}/core
)

add_library(${PROJECT_NAME}_CORE
  ${${PROJECT_NAME}_CORE_FILES}
)

# ####################### Tests ####################
add_executable(${PROJECT_NAME}_problem_test
  tests/problem_test.cc
)

target_link_libraries(
  ${PROJECT_NAME}_problem_test
  gtest_main
  gtest
  pthread
  ${PROJECT_NAME}_CORE
  glog
)

add_test(NAME ${PROJECT_NAME}_problem_test COMMAND ${PROJECT_NAME}_problem_test)

# gtest_discover_tests(${PROJECT_NAME}_problem_test)
