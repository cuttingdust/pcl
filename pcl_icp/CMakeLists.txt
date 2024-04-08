cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(pcl_icp)

find_package(PCL 1.4 REQUIRED COMPONENTS common io visualization registration)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
