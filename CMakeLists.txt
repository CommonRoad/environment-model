cmake_minimum_required(VERSION 3.16)
project(EnvironmentModel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile command database is required for Clang-assisted parsing in Doxygen
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

option(ENV_MODEL_BUILD_DOXYGEN "Build doxygen" OFF)
option(ENV_MODEL_BUILD_TESTS "Build tests" ON)
option(ENV_MODEL_BUILD_PYBIND "Build python interface" OFF)
option(ENV_MODEL_BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(ENV_MODEL_BUILD_CODE_COVERAGE "Build code coverage" OFF)
option(ENV_MODEL_BUILD_EXECUTABLE "Build executable" ON)

# Provides configure_package_config_file
include(CMakePackageConfigHelpers)
# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

# Appends dictionary where other cmake files are stored which install required Software
include(FetchContent)

FetchContent_Declare(
        external_cmake_modules

        GIT_REPOSITORY git@gitlab.lrz.de:cps/rapid-prototyping-setup.git
        GIT_TAG        fe64e917d3b0173c006e8ca04a6a8ed23c2292c7
        GIT_PROGRESS   true
        )

FetchContent_MakeAvailable(external_cmake_modules)

list(APPEND CMAKE_MODULE_PATH ${external_cmake_modules_SOURCE_DIR}/cmake)

# Set  optimizations for build types
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

include(extras/DiscoverLLD OPTIONAL)
include(extras/DiscoverSanitizers)

# Include Eigen3, Boost filesystem, spdlog, and OpenMP
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenMP REQUIRED)
set_target_properties(protobuf::protoc PROPERTIES IMPORTED_GLOBAL TRUE)


include(external/ExternalGoogleTest)
include(external/ExternalDrivabilityChecker)
include(external/ExternalPugixml)
include(external/ExternalPybind)
include(external/ExternalRange)
include(external/ExternalYamlCpp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(include/commonroad_cpp/interfaces/commonroad/protobufFormat)

# Add Environment model node
add_subdirectory(src)

if(ENV_MODEL_BUILD_EXECUTABLE)
    add_subdirectory(example)
endif(ENV_MODEL_BUILD_EXECUTABLE)

## Doxygen
if(ENV_MODEL_BUILD_DOXYGEN)
    include(extras/Doxygen)
endif(ENV_MODEL_BUILD_DOXYGEN)

# add unit testing
if(ENV_MODEL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(ENV_MODEL_BUILD_TESTS)


install(TARGETS env_model commonroad_protobuf yaml-cpp
        EXPORT ${PROJECT_NAME}_Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/external/${PROJECT_NAME}Config.cmake.in"
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
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

