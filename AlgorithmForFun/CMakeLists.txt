# Algorithm For Fun
project(AFF)
cmake_minimum_required(VERSION 2.2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
        main.cpp
        environment.cpp
        environment.h
        dfs.cpp
        dfs.h
        bcd.cpp
        bcd.h
        bfs.cpp
        bfs.h
        dijkstra.cpp
        dijkstra.h
        bcd_with_dijkstra.cpp
        bcd_with_dijkstra.h hybrid_astar.cpp hybrid_astar.h transformer.cpp transformer.h typedefine.h)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread -lm)