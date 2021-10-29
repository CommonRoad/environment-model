cmake_minimum_required(VERSION 3.16)
project(EnvironmentModel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DOXYGEN "Build doxygen" OFF)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_PYBIND "Build python interface" OFF)

# Appends dictionary where other cmake files are stored which install required Software
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

# Set compiler optimizations for build types
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")

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


add_subdirectory(external)

find_package(DrivabilityChecker REQUIRED CONFIG)

# Include Eigen3, Boost filesystem, and OpenMP
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
    message(SEND_ERROR "Can't find the OpenMP package")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


## Include pybind
if(BUILD_PYBIND)
    add_subdirectory(external/pybind11)
    set(PYBIND11_CPP_STANDARD -std=c++17)
endif(BUILD_PYBIND)

# Add Environment model node
add_subdirectory(src)

## Doxygen
if(BUILD_DOXYGEN)
    include(doxygen)
endif(BUILD_DOXYGEN)

# add unit testing
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(BUILD_TESTS)

# Provides configure_package_config_file
include(CMakePackageConfigHelpers)

# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

install(TARGETS env_model
    EXPORT ${PROJECT_NAME}_Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(BUILD_PYBIND)
    install(TARGETS cpp_env_model env_model
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
    )
endif(BUILD_PYBIND)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Export target configuration (for installation)
install(EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Export target configuration (for local building)
export(EXPORT ${PROJECT_NAME}_Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  )

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.c" PATTERN "*.cpp"
  )


