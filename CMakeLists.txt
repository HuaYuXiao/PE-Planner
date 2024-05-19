cmake_minimum_required(VERSION 3.0)
project(pe_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx -mfma -ffast-math -march=native")

find_package(Python3 COMPONENTS Development NumPy)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roslib
        roscpp
        tf
)

include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        /usr/lib/include
        ${PROJECT_SOURCE_DIR}/include
        modules
)
link_directories(/usr/local/lib)

add_executable(planner_px4
    src/planner_px4_main.cpp
    modules/mpcc/nominal_mpcc.cpp
    modules/map/map.cpp
    modules/ros_interface/ros_interface.cpp
    modules/kinodynamic_astar/kinodynamic_astar.cpp
    modules/bspline_opt/bspline_optimizer.cpp
    modules/bspline_opt/bspline_optimizer2.cpp
    modules/px4_interface/px4_interface.cpp)
target_link_libraries(planner_px4
    Python3::Python
        Python3::NumPy
        nlopt
        ${catkin_LIBRARIES}
)

add_executable(map_viewer
    src/map_viewer.cpp
    modules/map/map.cpp
    modules/ros_interface/ros_interface.cpp)
target_link_libraries(map_viewer
    ${catkin_LIBRARIES}
)

add_executable(map_generator
    src/map_generator.cpp
    modules/map/map.cpp
    modules/ros_interface/ros_interface.cpp)
target_link_libraries(map_generator
    ${catkin_LIBRARIES}
)
