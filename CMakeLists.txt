cmake_minimum_required(VERSION 3.0)
project(pe_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Python3 COMPONENTS Development NumPy)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roslib
        roscpp
        tf
        plan_env
        bspline_opt
        path_searching
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        /usr/lib/include
        modules
)

link_directories(/usr/local/lib)

add_executable(planner_px4
    src/planner_px4_main.cpp
    modules/mpcc/nominal_mpcc.cpp
    modules/ros_interface/ros_interface.cpp
    modules/px4_interface/px4_interface.cpp
)
target_link_libraries(planner_px4
    Python3::Python
        Python3::NumPy
        nlopt
        ${catkin_LIBRARIES}
)
