cmake_minimum_required(VERSION 3.5)

project(controller)

find_package(Boost COMPONENTS system thread filesystem)
find_package(urdf REQUIRED)

add_definitions(-std=c++17)

include(FindPkgConfig)
pkg_check_modules(cnoid_plugin REQUIRED choreonoid-body-plugin)

include_directories(
	include
	${cnoid_plugin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${cnoid_plugin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

set(MOTION_ENGINE_SRC
	src/motion_engine.cpp
	src/kinematics.cpp
	src/link.cpp
)

add_library(controller SHARED src/controller.cpp ${MOTION_ENGINE_SRC})
set_target_properties(controller PROPERTIES PREFIX "")

set(target controller)
target_link_libraries(controller
	${cnoid_plugin_LIBRARIES}
	${Boost_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

list(APPEND project_files controller)

install(TARGETS ${project_files} LIBRARY DESTINATION /usr/local/lib/choreonoid-1.8/simplecontroller)
