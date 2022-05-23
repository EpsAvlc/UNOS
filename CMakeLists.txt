cmake_minimum_required(VERSION 3.0.0)
project(UNOS VERSION 0.1.0)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "DEBUG")

include_directories(
  /usr/include/eigen3
  ${CMAKE_CURRENT_LIST_DIR}/include
)


####################### Tests ####################
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(${PROJECT_NAME}_test
  tests/test.cc
)

target_link_libraries(
  ${PROJECT_NAME}_test
  gtest_main
)

gtest_discover_tests(${PROJECT_NAME}_test)
