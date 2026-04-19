cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(navigation 
  src/navigation.cpp 

  src/grid/global_costmap.cpp 
  src/grid/local_costmap.cpp 
  src/grid/grid.cpp

  src/geometry/pose.cpp
  src/geometry/state.cpp
  src/geometry/collision_checker.cpp

  src/planner/hybrid_astar.cpp
  src/planner/reed_shepps.cpp
  src/planner/dubins.cpp
  src/planner/motion_model.cpp
  src/planner/optimizer.cpp

  src/controller/regulated_pure_pursuit.cpp

  src/utils/DSU.cpp
  src/utils/EDT.cpp
  src/utils/math_utils.cpp
  src/utils/grid_utils.cpp
  src/utils/trig_utils.cpp)
target_include_directories(navigation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(navigation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  navigation
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS navigation
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
