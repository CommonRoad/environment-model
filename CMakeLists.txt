cmake_minimum_required(VERSION 3.16)
project(env_model)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Appends dictionary where other cmake files are stored which install required Software
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
    set(default_build_type "Debug")
endif()
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
endif()

# Activate code coverage
if (CODE_COVERAGE)
    option(CodeCoverage "CodeCoverage" ON)
endif()

# Include Curvilinear Coordinate System
if(NOT EXISTS "${CRCCOSY_LIBRARY_DIR}")
    message(SEND_ERROR "Can't find the CommonRoad-Curvilinear-Coordinate System library in ${CRCCOSY_LIBRARY_DIR}")
endif()
set(CRCCOSY_LIBRARY ${CRCCOSY_LIBRARY_DIR}/libcrccosy.a)
set(LIB_CRCCOSY_INCLUDE ${CRCCOSY_LIBRARY_DIR}/cpp)

# Include Eigen3, Boost filesystem, and OpenMP
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
    message(SEND_ERROR "Can't find the OpenMP package")
endif()

# Set compiler optimizations for build types
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../)

# Add Environment model node
add_subdirectory(src)

# Doxygen
include(doxygen)

# add unit testing
enable_testing()
add_subdirectory(tests)





