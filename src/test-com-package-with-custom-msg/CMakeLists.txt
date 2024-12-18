cmake_minimum_required(VERSION 3.8)
project(test-com-package-with-custom-msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(sbus_interface REQUIRED)
# <depend>rclcpp</depend>
# <depend>std_msgs</depend>
# <depend>custom_interfaces</depend>

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(dual-nodes src/dual-nodes.cpp)
# 节点可执行文件和节点代码
add_executable(talker-listener src/pub-sub-function-with-msg.cpp)
add_executable(steering-cmd-sim src/steering-cmd.cpp)
add_executable(steering-cmd-user src/steering-with-userinput.cpp)
add_executable(steering-cmd-sbus src/steering-with-sbus.cpp)


ament_target_dependencies(talker-listener rclcpp std_msgs custom_interfaces)
ament_target_dependencies(steering-cmd-sim rclcpp std_msgs custom_interfaces)
ament_target_dependencies(steering-cmd-user rclcpp custom_interfaces)
ament_target_dependencies(steering-cmd-sbus rclcpp custom_interfaces sbus_interface)

# target_include_directories(dual-nodes PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# target_compile_features(dual-nodes PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  # dual-nodes
  talker-listener
  steering-cmd-sim
  steering-cmd-user
  steering-cmd-sbus
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
