cmake_minimum_required(VERSION 3.30)
project(RouteSys_Project)

set(BOOST_ROOT "E:/something/boost_1_71_0")
include_directories("${BOOST_ROOT}")

# 四个 .a（写“完整文件名”最稳）
set(BOOST_LIB_DIR "${BOOST_ROOT}/stage/lib")
set(BOOST_THREAD   "${BOOST_LIB_DIR}/libboost_thread-mgw151-mt-s-x64-1_71.a")
set(BOOST_CHRONO   "${BOOST_LIB_DIR}/libboost_chrono-mgw151-mt-s-x64-1_71.a")
set(BOOST_FILESYS  "${BOOST_LIB_DIR}/libboost_filesystem-mgw151-mt-s-x64-1_71.a")
set(BOOST_SYSTEM   "${BOOST_LIB_DIR}/libboost_system-mgw151-mt-s-x64-1_71.a")

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

target_link_libraries(RouteSys_Project PRIVATE
${BOOST_THREAD} ${BOOST_CHRONO} ${BOOST_FILESYS} ${BOOST_SYSTEM}
)
      
target_link_options(RouteSys_Project PRIVATE -static -static-libgcc -static-libstdc++)
