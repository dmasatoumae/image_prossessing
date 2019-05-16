cmake_minimum_required(VERSION 3.5.1)
project(realsence)
set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main ${realsense2_LIBRARY})


