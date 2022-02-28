cmake_minimum_required(VERSION 3.20)
project(BulletSynthesis)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(dependencies/bullet3)
add_library(BulletSynthesis SHARED src/NativeExport.cpp src/PhysicsManager.cpp src/PhysicsManager.h)

set_target_properties(BulletSynthesis
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

target_include_directories(BulletSynthesis PUBLIC dependencies/bullet3/src)
target_link_libraries(BulletSynthesis ${BULLET_LIBRARIES})
