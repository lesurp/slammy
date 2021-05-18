cmake_minimum_required(VERSION 3.0.2)
project(slammy)
catkin_package()

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
add_compile_definitions(SPDLOG_FMT_EXTERNAL)
add_compile_definitions(OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)
include_directories(${catkin_INCLUDE_DIRS})

set(slammy_deps ${catkin_LIBRARIES} fmt spdlog OpenMP::OpenMP_CXX)

add_library(slammy_lib
    src/tracker.cpp
    src/vision.cpp
    src/ros_bridge.cpp
    src/utils/ass.cpp
    src/utils/debug.cpp
)
target_link_libraries(slammy_lib ${slammy_deps})

if(CATKIN_ENABLE_TESTING)
    function(register_test name)
        add_rostest_gtest(test_${name} test/test_${name}.launch
            test/test_${name}.cpp ${ARGN})
        target_link_libraries(test_${name} slammy_lib gtest_main)
        target_compile_definitions(test_${name} PUBLIC SLAMMY_NO_DBG)
    endfunction()

    find_package(rostest REQUIRED)
    register_test(relative_pose)
    register_test(triangulation)
endif()

add_executable(slammy src/slammy.cpp)
target_link_libraries(slammy slammy_lib)
