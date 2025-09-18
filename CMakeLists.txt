cmake_minimum_required(VERSION 3.30)
project(RouteSys_Project)

find_package(Boost 1.71.0 COMPONENTS thread system filesystem REQUIRED)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
else()
    message(FATAL_ERROR "Boost not found")
endif ()

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")

add_executable(RouteSys_Project main.cpp
        graph.cpp
        graph.h
        simulation.cpp
        simulation.h
        RR-Index.cpp
        Insert_data_generation.cpp
        insertion.cpp
        temp.cpp)
