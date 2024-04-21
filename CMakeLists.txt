cmake_minimum_required(VERSION 3.8)
project(arcanain_modern_control_tutorial)

# Set the RPATH
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(EXTERNAL_LIB_HEADERS "cpp_robotics")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(std_msgs REQUIRED)

# 01_unforced_response_with_mass_spring_damper
add_executable(unforced_response_with_mass_spring_damper src/01_unforced_response_with_mass_spring_damper.cpp)
ament_target_dependencies(unforced_response_with_mass_spring_damper rclcpp std_msgs Eigen3)

# feedback_control_of_siso_system
add_executable(feedback_control_of_siso_system src/02_feedback_control_of_siso_system.cpp)
ament_target_dependencies(feedback_control_of_siso_system rclcpp std_msgs Eigen3)

# feedback_control_of_siso_system
add_executable(optimal_control_using_lqr src/03_optimal_control_using_lqr.cpp)
ament_target_dependencies(optimal_control_using_lqr rclcpp std_msgs Eigen3)

add_subdirectory(cpp_robotics)

# Install Cpp executables
install(TARGETS
  unforced_response_with_mass_spring_damper
  feedback_control_of_siso_system
  optimal_control_using_lqr
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp_robotics/include
)

target_include_directories(
  feedback_control_of_siso_system PRIVATE
  ${EXTERNAL_LIB_HEADERS}
)

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

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
