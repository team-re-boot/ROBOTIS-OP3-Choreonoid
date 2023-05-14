cmake_minimum_required(VERSION 3.5)

project(VisionController)

find_package(Boost COMPONENTS system thread filesystem)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++17)

include(FindPkgConfig)
pkg_check_modules(cnoid_plugin REQUIRED choreonoid-body-plugin)

include_directories(
	include
	${cnoid_plugin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${cnoid_plugin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

add_library(VisionController SHARED VisionController.cpp ${MOTION_ENGINE_SRC} )
set_target_properties(VisionController PROPERTIES PREFIX "")

set(target VisionController)
target_link_libraries(VisionController
	${cnoid_plugin_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

install(TARGETS VisionController LIBRARY DESTINATION /usr/local/lib/choreonoid-1.8/simplecontroller)
