cmake_minimum_required(VERSION 3.8)
project(linear_algebra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tech_assess_msgs REQUIRED) #declare dependency to tech_assess_msgs
find_package(Eigen3 REQUIRED) #declare dependency to Eigen
find_package(yaml-cpp REQUIRED)

#add executable
add_executable(simple_publisher src/simple_publisher.cpp)
add_executable(simple_subscriber src/simple_subscriber.cpp)
add_executable(least_squares_server src/least_squares_server.cpp)
add_executable(least_squares_client src/least_squares_client.cpp)

# find header files for eigen
target_include_directories(least_squares_server PUBLIC /usr/include/eigen3)
target_include_directories(least_squares_client PUBLIC /usr/include/eigen3)

#declare dependencies to the executable
ament_target_dependencies(simple_publisher rclcpp std_msgs)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)
ament_target_dependencies(least_squares_server rclcpp tech_assess_msgs Eigen3 geometry_msgs)
ament_target_dependencies(least_squares_client rclcpp tech_assess_msgs Eigen3 yaml-cpp geometry_msgs)

# link compiled code with yaml library
target_link_libraries(least_squares_client yaml-cpp)

# Copy config files to install
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#install publisher script
install(TARGETS simple_publisher
        DESTINATION lib/${PROJECT_NAME}) 
#install subscriber script
install(TARGETS simple_subscriber
        DESTINATION lib/${PROJECT_NAME})
#install least_squares_server script
install(TARGETS least_squares_server
        DESTINATION lib/${PROJECT_NAME})
#install least_squares_client script
install(TARGETS least_squares_client
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
