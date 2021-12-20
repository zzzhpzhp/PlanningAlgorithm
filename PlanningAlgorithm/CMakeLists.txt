project(PlanningAlgorithm)
cmake_minimum_required(VERSION 2.2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

file(GLOB SRC_FILES
        ./utility/*.cpp
        ./algorithms/*.cpp
        ./environment/*.cpp
        ./*.cpp
        )
include_directories(
        ./
        ./utility
        ./interface
        ./algorithms
        ./environment
)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
        -lpthread -lm)