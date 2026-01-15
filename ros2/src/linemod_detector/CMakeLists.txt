cmake_minimum_required(VERSION 3.8)
project(linemod_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D GLM_ENABLE_EXPERIMENTAL=TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(OpenGL REQUIRED)

set(LINEMOD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)

file(GLOB_RECURSE LINEMOD_SOURCES
  ${LINEMOD_ROOT}/src/*.cpp)

list(FILTER LINEMOD_SOURCES EXCLUDE REGEX ".*/Kinect2\\.cpp$")
list(FILTER LINEMOD_SOURCES EXCLUDE REGEX ".*/Aruco\\.cpp$")

add_executable(linemod_detector_node
  src/linemod_detector_node.cpp
  ${LINEMOD_SOURCES}
)

target_include_directories(linemod_detector_node PUBLIC
  ${LINEMOD_ROOT}/include
)

ament_target_dependencies(linemod_detector_node
  rclcpp
  sensor_msgs
  message_filters
)

target_link_libraries(linemod_detector_node
  ${OpenCV_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  ${GLEW_LIBRARIES}
  stdc++fs
)

if (ASSIMP_LIBRARIES)
  target_link_libraries(linemod_detector_node ${ASSIMP_LIBRARIES})
else ()
  find_library(ASSIMP_LIBRARY NAMES assimp)
  target_link_libraries(linemod_detector_node ${ASSIMP_LIBRARY})
endif ()

if (TARGET SDL2::SDL2)
  target_link_libraries(linemod_detector_node SDL2::SDL2)
else()
  target_link_libraries(linemod_detector_node ${SDL2_LIBRARIES})
  target_include_directories(linemod_detector_node PUBLIC ${SDL2_INCLUDE_DIR})
endif()

ament_package()

install(TARGETS linemod_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
