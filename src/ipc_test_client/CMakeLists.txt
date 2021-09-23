cmake_minimum_required(VERSION 3.5)
project(ipc_test_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ipc_test_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    ipc_test_msgs
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
