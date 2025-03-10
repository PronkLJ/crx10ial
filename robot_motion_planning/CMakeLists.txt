cmake_minimum_required(VERSION 3.8)
project(robot_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable for adding ground plane
add_executable(add_ground_plane src/add_ground_plane.cpp)
target_include_directories(add_ground_plane PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(add_ground_plane
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add executable for adding ceiling plane
add_executable(add_ceiling_plane src/add_ceiling_plane.cpp)
target_include_directories(add_ceiling_plane PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(add_ceiling_plane
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add static_move_program executable
add_executable(static_move_program src/static_move_program.cpp)
target_include_directories(static_move_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(static_move_program PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  static_move_program
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add dynamic_move_program executable
add_executable(dynamic_move_program src/dynamic_move_program.cpp)
target_include_directories(dynamic_move_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_move_program PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dynamic_move_program
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add executable to move robot to homing position
add_executable(move_to_home src/move_to_home.cpp)
target_include_directories(move_to_home PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_home PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_home
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS add_ground_plane add_ceiling_plane static_move_program dynamic_move_program move_to_home
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
