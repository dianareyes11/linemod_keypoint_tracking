cmake_minimum_required(VERSION 3.0.0)
project(LINE-MOD VERSION 0.1.0)

include_directories(${CMAKE_SOURCE_DIR}/include/)
add_definitions(-D GLM_ENABLE_EXPERIMENTAL=TRUE)
option(USE_FREENECT2 "Enable Kinect v2 support via libfreenect2" OFF)
option(BUILD_ARUCO_BENCHMARK "Build Aruco benchmark tool" OFF)
# Add source files
file(GLOB_RECURSE SOURCE_FILES 
	${CMAKE_SOURCE_DIR}/src/*.c
	${CMAKE_SOURCE_DIR}/src/*.cpp)

# Add header files
file(GLOB_RECURSE HEADER_FILES 
	${CMAKE_SOURCE_DIR}/include/*.h
	${CMAKE_SOURCE_DIR}/include/*.hpp)

if (NOT USE_FREENECT2)
	list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/Kinect2\\.cpp$")
else ()
	add_definitions(-DUSE_FREENECT2)
endif ()

if (NOT BUILD_ARUCO_BENCHMARK)
	list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/Aruco\\.cpp$")
endif ()


macro (add_executable _name)
	_add_executable(${ARGV})
	# SDL2
	find_package(SDL2 REQUIRED)
	if (TARGET SDL2::SDL2)
		target_link_libraries(${_name} SDL2::SDL2)
	else ()
		target_link_libraries(${_name} ${SDL2_LIBRARIES})
		target_include_directories(${_name}  PUBLIC ${SDL2_INCLUDE_DIR})
	endif ()

	# GLM
	find_package(glm REQUIRED)
	target_link_libraries(${_name}  ${glm_LIBRARY})
	target_include_directories(${_name}  PUBLIC ${glm_INCLUDE_DIR})

	# Assimp
	find_package(ASSIMP REQUIRED)
	if (ASSIMP_LIBRARIES)
		set(ASSIMP_LIBRARY ${ASSIMP_LIBRARIES})
	else ()
		find_library(ASSIMP_LIBRARY NAMES assimp PATHS ${ASSIMP_LIBRARY_DIRS})
	endif ()
	target_link_libraries(${_name}  ${ASSIMP_LIBRARY})
	target_include_directories(${_name}  PUBLIC ${ASSIMP_INCLUDE_DIR})

	# OpenGL
	find_package(OpenGL REQUIRED)
	target_link_libraries(${_name}  ${OPENGL_gl_LIBRARY})
	target_include_directories(${_name}  PUBLIC ${OPENGL_INCLUDE_DIR})

	#libfreenect2
	if (USE_FREENECT2)
		set(CMAKE_PREFIX_PATH "~/freenect2/lib/cmake/freenect2/")
		#set(freenect2_DIR ${HOME}/freenect2/lib/cmake/freenect2)
		find_package(freenect2 REQUIRED)
		target_link_libraries(${_name}  ${freenect2_LIBRARY})
		target_include_directories(${_name}  PUBLIC ${freenect2_INCLUDE_DIR})
	endif ()

	# GL Extension Wrangler (GLEW)
	find_package(GLEW REQUIRED)
	target_link_libraries(${_name}  ${GLEW_LIBRARIES})
	target_include_directories(${_name}  PUBLIC ${GLEW_INCLUDE_DIRS})

	# OpenCV
	find_package(OpenCV REQUIRED)
	target_link_libraries(${_name}  ${OpenCV_LIBRARIES})
	target_include_directories(${_name}  PUBLIC ${OpenCV_INCLUDE_DIRS})

	target_link_libraries(${_name}  stdc++fs)
	add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

	set_property(TARGET ${_name}  PROPERTY CXX_STANDARD 17)
	set_property(TARGET ${_name}  PROPERTY CXX_STANDARD_REQUIRED ON)

endmacro()



add_executable(Detector detector.cpp ${HEADER_FILES} ${SOURCE_FILES})
add_executable(Template_Generator templateGeneration.cpp ${HEADER_FILES} ${SOURCE_FILES})
if (BUILD_ARUCO_BENCHMARK)
	add_executable(Arcuo_Benchmark arucoBenchmark.cpp ${HEADER_FILES} ${SOURCE_FILES})
endif ()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
