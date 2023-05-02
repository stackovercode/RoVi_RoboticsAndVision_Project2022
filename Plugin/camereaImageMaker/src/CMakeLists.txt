# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project(DynamicDrawable)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(getCameraView main.cpp)
target_link_libraries(getCameraView ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${OpenCV_LIBS})
