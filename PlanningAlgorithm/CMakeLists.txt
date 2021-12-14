project(PlanningAlgorithm)
cmake_minimum_required(VERSION 2.2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
file(GLOB SRC_FILES *.cpp)
include_directories(./)
add_executable(${PROJECT_NAME}
        ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread -lm)