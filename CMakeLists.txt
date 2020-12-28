cmake_minimum_required(VERSION 3.16)
project(src)

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

# Set compiler optimizations for build types
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

# Add Environment model node
add_subdirectory(src)

# Doxygen
include(doxygen)

# add unit testing
enable_testing()
add_subdirectory(tests)
add_subdirectory(external/googletest)





