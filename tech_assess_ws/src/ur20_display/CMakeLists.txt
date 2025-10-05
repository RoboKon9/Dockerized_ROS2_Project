cmake_minimum_required(VERSION 3.8)
project(ur20_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
#add executable
add_executable(state_publisher src/custom_state_publisher.cpp)

#declare dependencies to the executable
ament_target_dependencies(state_publisher rclcpp yaml-cpp sensor_msgs tf2 tf2_ros tf2_eigen geometry_msgs rviz_visual_tools)

#install custom_state_publisher script 
install(TARGETS state_publisher
        DESTINATION lib/${PROJECT_NAME})

# read header files for Eigen
target_include_directories(state_publisher PUBLIC /usr/include/eigen3)

#link executable to external ros library
target_link_libraries(state_publisher Eigen3::Eigen )

# copy launch folder to install
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# copy config folder to install
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# copy rviz folder to install
install(DIRECTORY rviz
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
