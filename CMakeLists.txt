cmake_minimum_required(VERSION 3.0.0)
project(UNOS VERSION 0.1.0)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "DEBUG")

find_package(GTest REQUIRED)

add_compile_options(-std=c++17)
include_directories(
  /usr/include/eigen3
  ${CMAKE_CURRENT_LIST_DIR}/include
)


####################### Tests ####################
add_executable(${PROJECT_NAME}_test
  tests/test.cc
)

target_link_libraries(
  ${PROJECT_NAME}_test
  gtest_main
  gtest
  pthread
)

gtest_discover_tests(${PROJECT_NAME}_test)
