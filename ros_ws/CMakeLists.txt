cmake_minimum_required(VERSION 3.0.2)
project(sensor_alerta)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


add_executable(alerta_automatica src/alerta_automatica.cpp)
target_link_libraries(alerta_automatica ${catkin_LIBRARIES})
add_dependencies(alerta_automatica ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


