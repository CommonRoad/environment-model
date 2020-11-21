project(env_model)

cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-ignored-attributes)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

if(NOT EXISTS "${CRCCOSY_LIBRARY_DIR}")
    message(SEND_ERROR "Can't find the CommonRoad-Curvilinear-Coordinate System library in ${CRCCOSY_LIBRARY_DIR}")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

set(CRCCOSY_LIBRARY ${CRCCOSY_LIBRARY_DIR}/libcrccosy.a)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

add_subdirectory(env_model)

