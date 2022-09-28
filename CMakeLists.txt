cmake_minimum_required(VERSION 3.0.0)
project(UNOS VERSION 0.1.0)

enable_testing()
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "DEBUG")

file(GLOB_RECURSE ${PROJECT_NAME}_CORE_FILES "${CMAKE_CURRENT_LIST_DIR}/core/*.cc")

find_package(GTest REQUIRED)
find_package(Ceres REQUIRED)
include_directories(
  /usr/local/include/eigen3
  ${CMAKE_CURRENT_LIST_DIR}/core
  ${CMAKE_CURRENT_LIST_DIR}
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_CORE
  ${${PROJECT_NAME}_CORE_FILES}
)

# ####################### Tests ####################
file(GLOB_RECURSE TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/tests/*.cc")
add_executable(${PROJECT_NAME}_test
  ${TEST_FILES}
)

target_link_libraries(
  ${PROJECT_NAME}_test
  gtest_main
  gtest
  pthread
  ${PROJECT_NAME}_CORE
  glog
  ${CERES_LIBRARIES}
)

gtest_discover_tests(${PROJECT_NAME}_test)
