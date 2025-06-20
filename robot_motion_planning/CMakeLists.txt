cmake_minimum_required(VERSION 3.8)
project(robot_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# Add ur_reaction executable
add_executable(ur_reaction src/ur_reaction.cpp)
target_include_directories(ur_reaction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur_reaction PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur_reaction
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add subscribe_and_move executable
add_executable(subscribe_and_move src/subscribe_and_move.cpp)
target_include_directories(subscribe_and_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(subscribe_and_move PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  subscribe_and_move
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add pose_publisher executable
add_executable(pose_publisher src/pose_publisher.cpp)
target_include_directories(pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pose_publisher
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
)

# Add crx_publisher executable
add_executable(crx_publisher src/crx_publisher.cpp)
target_include_directories(crx_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crx_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(crx_publisher
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
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

# Add executable to CLF demo
add_executable(clf_demo src/clf_demo.cpp)
target_include_directories(clf_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(clf_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  clf_demo
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Add executable to object pickup
add_executable(object_pickup src/object_pickup.cpp)
target_include_directories(object_pickup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(object_pickup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  object_pickup
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS add_ground_plane add_ceiling_plane static_move_program dynamic_move_program move_to_home clf_demo object_pickup subscribe_and_move pose_publisher crx_publisher ur_reaction
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
