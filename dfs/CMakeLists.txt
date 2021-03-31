# Algorithm For Fun
project(AFF)
cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}  main.cpp environment.cpp environment.h dfs.cpp dfs.h bfs.cpp bfs.h)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread -lm)