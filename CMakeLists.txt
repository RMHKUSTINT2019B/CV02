cmake_minimum_required(VERSION 3.9)
project(CV02)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(LibUSB REQUIRED)
find_package(Threads REQUIRED)

file(GLOB_RECURSE SRC Source/*)

add_executable(CV02 ${SRC})

target_include_directories(CV02 PUBLIC ${OpenCV_INCLUDE_DIRS} ${LibUSB_INCLUDE_DIRS})
target_link_libraries(CV02 ${OpenCV_LIBS} ${LibUSB_LIBRARIES} Threads::Threads)
