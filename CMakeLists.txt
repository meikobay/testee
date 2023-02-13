cmake_minimum_required(VERSION 2.8.3)
PROJECT (TESTEE)
SET(CMAKE_BUILD_TYPE "Release")
set(TOP_DIR ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TOP_DIR}/)

include_directories(${TOP_DIR}/include/)
link_directories(${TOP_DIR}/libs/)

add_executable(testee main.cpp)
target_link_libraries(testee VTSMapInterfaceCPP)
