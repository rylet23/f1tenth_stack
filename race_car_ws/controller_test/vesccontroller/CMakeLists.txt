cmake_minimum_required(VERSION 3.8)
project(vesc_controller_cpp)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Try finding Boost with component specification
find_package(Boost REQUIRED COMPONENTS system)

add_executable(vesc_node src/vesc_node.cpp)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  target_link_libraries(vesc_node ${Boost_LIBRARIES})
endif()

ament_target_dependencies(vesc_node rclcpp std_msgs)

install(TARGETS vesc_node
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch)

# install(DIRECTORY resource
#   DESTINATION share/${PROJECT_NAME}/resource)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
