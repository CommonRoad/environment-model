cmake_minimum_required(VERSION 3.20..3.24)
project(EnvironmentModel
    LANGUAGES C CXX
    VERSION 2022.0
    HOMEPAGE_URL "https://commonroad.in.tum.de/"
    DESCRIPTION "Classes and methods to represent the CommonRoad format in C++17")

set(CMAKE_COLOR_DIAGNOSTICS ON)

if(SKBUILD)
        # Scikit-Build does not add your site-packages to the search path
        # automatically, so we need to add it _or_ the pybind11 specific directory
        # here.
        execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c
                "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE _tmp_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ECHO STDOUT)
        list(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")

        # set(Python_FIND_VIRTUALENV ONLY)

        set(PYBIND11_FINDPYTHON ON)
        find_package(Python 3.7 REQUIRED COMPONENTS Development.Module)

        # Now we can find pybind11
        find_package(pybind11 2.7.0 CONFIG REQUIRED)
endif()

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

if(DEFINED ENV{CIBUILDWHEEL})
    # include(ci/ccache.cmake)
    # include(ci/sccache.cmake)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Extra debugging for project developers - safe to disable
set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)
set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

# Compile command database is required for Clang-assisted parsing in Doxygen
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

option(BUILD_SHARED_LIBS "Build environment model as a shared library" ON)

option(ENV_MODEL_USE_LLD "Use lld if possible" ON)
option(ENV_MODEL_BUILD_DOXYGEN "Build doxygen" OFF)
option(ENV_MODEL_BUILD_TESTS "Build tests" ON)
# TODO Eigen CMake has a nice approach to setting per-project BUILD_SHARED_LIBS
# option(ENV_MODEL_BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(ENV_MODEL_BUILD_CODE_COVERAGE "Build code coverage" OFF)
option(ENV_MODEL_BUILD_EXECUTABLE "Build executable" ON)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
cmake_policy(SET CMP0077 NEW)
cmake_policy(SET CMP0126 NEW)

if(SKBUILD)
  message(STATUS "PYTHON MODE - assuming we are invoked by pip/setup.py")
  message(STATUS "PYTHON MODE - building static libraries")
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(PYBIND11_FINDPYTHON ON)

  #set(ENV_MODEL_BUILD_EXECUTABLE OFF)
  set(CR_USE_SYSTEM_YAMLCPP OFF CACHE BOOL "" FORCE)
  set(CR_USE_SYSTEM_PUGIXML OFF CACHE BOOL "" FORCE)

  set(ENV_MODEL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ENV_MODEL_BUILD_DOXYGEN OFF CACHE BOOL "" FORCE)
  set(ENV_MODEL_BUILD_EXECUTABLE OFF CACHE BOOL "" FORCE)

  set(FETCHCONTENT_QUIET ON)

  set(ADD_PYTHON_BINDINGS ON CACHE INTERNAL "" FORCE)

  cmake_policy(SET CMP0063 NEW)

  set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
  set(CMAKE_CXX_VISIBILITY_PRESET default)
endif()

# Provides configure_package_config_file
include(CMakePackageConfigHelpers)
# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

# Set  optimizations for build types
# set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
# set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")

# Set a default build type if none was specified
if(OFF)
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
    set(default_build_type "Debug")
endif()
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
endif()
endif()

include(FetchContent)
FetchContent_Declare(
    commonroad_cmake

    GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
    GIT_TAG        feature/python
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

include(toolchain/DiscoverLLD OPTIONAL)
include(toolchain/DiscoverSanitizers OPTIONAL)

include(extras/GitIgnoreBinaryDir OPTIONAL)

if(DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external_v2)

#if(${CMAKE_VERSION} VERSION_LESS "3.24.0")
# Enable this for debugging Boost discovery
# set(Boost_DEBUG ON)
find_package(Boost 1.70 OPTIONAL_COMPONENTS program_options filesystem)
if(NOT Boost_FOUND)
    message(STATUS "Boost - falling back to external version")
    include(ExternalBoost)
    else()
        message(STATUS "Boost - SYSTEM")
endif()

option(COMMONROAD_SYSTEM_PROTOBUF "Use system Protobuf" ON)
# set(Protobuf_DEBUG ON)
if(DEFINED ENV{CI})
    # Ugly hack, as the CI environment has another protoc in /root/anaconda3/bin/protoc
    # which is incompatible with the Protobuf libraries found by CMake
    set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE PATH "The protoc compiler" FORCE)
endif()

if(COMMONROAD_SYSTEM_PROTOBUF)
    find_package(Protobuf 3.6)
endif()
if(NOT COMMONROAD_SYSTEM_PROTOBUF OR NOT Protobuf_FOUND)
    message(STATUS "Protobuf - falling back to external version")
    include(ExternalProtobuf)
else()
    message(STATUS "Protobuf - SYSTEM")

    # Check Protobuf compiler version to be aligned with libraries version
    execute_process(COMMAND ${Protobuf_PROTOC_EXECUTABLE} --version
                    OUTPUT_VARIABLE _PROTOBUF_PROTOC_EXECUTABLE_VERSION)

    if("${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}" MATCHES "libprotoc ([0-9.]+)")
        set(_PROTOBUF_PROTOC_EXECUTABLE_VERSION "${CMAKE_MATCH_1}")
    endif()

    if(Protobuf_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
            "${Protobuf_PROTOC_EXECUTABLE} reveals version ${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}")
    endif()

    if(NOT "${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}" VERSION_EQUAL "${Protobuf_VERSION}")
        message(FATAL_ERROR "Unfortunately, the system protobuf installation is unusable "
            "due to a detected mismatch between the Protobuf compiler and the Protobuf libraries. "
            "We can't proceed at this point since CMake already added the corresponding targets, "
            "and there is no way to remove those targets now. "
            "Please rerun the configuration, adding the -DCOMMONROAD_SYSTEM_PROTOBUF=OFF to the "
            "CMake command line.")
    endif()
endif()

if(DEFINED ENV{CIBUILDWHEEL})
    set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()

# Include Eigen3, Boost filesystem, spdlog, and OpenMP
if(DEFINED ENV{CIBUILDWHEEL} OR DEFINED ENV{CI} OR ${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.24.0")
    include(ExternalEigen)
    include(ExternalPugixml)
    include(ExternalSpdlog)
    include(ExternalGoogleTest)

    # IMPORTANT: crdc needs to be installed *before* yaml-cpp because yaml-cpp will generate a duplicate uninstall target otherwise
    include(ExternalDrivabilityChecker)
    include(ExternalYamlCpp)
else()
    message(WARNING "Automatic fallback to external versions not yet implemented for CMake < 3.24.0")

    find_package(Eigen3 REQUIRED)
    find_package(pugixml 1.11 REQUIRED)
    find_package(spdlog 1.8.0 REQUIRED)
    find_package(GTest)
    find_package(yaml-cpp 0.6.0 REQUIRED)

    # yaml-cpp::yaml-cpp is not present in installed config file as of version 0.7.0
    if(NOT TARGET yaml-cpp::yaml-cpp)
        add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    endif()

    include(ExternalDrivabilityChecker)
endif()

include(ExternalRange)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(include/commonroad_cpp/interfaces/commonroad/protobufFormat)

# Add Environment model node
add_subdirectory(src)

if(ENV_MODEL_BUILD_EXECUTABLE AND TARGET Boost::program_options)
    add_subdirectory(example)
elseif(NOT TARGET Boost::program_options)
    message(WARNING "disabling example because Boost::program_options was not found")
endif()

## Doxygen
if(ENV_MODEL_BUILD_DOXYGEN)
    include(extras/Doxygen)
endif(ENV_MODEL_BUILD_DOXYGEN)

# add unit testing
if(ENV_MODEL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(ENV_MODEL_BUILD_TESTS)


install(TARGETS commonroad_protobuf
    EXPORT ${PROJECT_NAME}_Targets
    FILE_SET commonroad_protobuf_headers)

install(TARGETS env_model env_model_core env_model_predicates
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
# FIXME: Does not work for now
# export(EXPORT ${PROJECT_NAME}_Targets
#         FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
#         NAMESPACE ${PROJECT_NAME}::
#         )

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


# TODO: Re-enable this and check whether things break
# include(utils/EnsureStatic)

# Sanity check: Ensure we are building all dependencies as static libraries
if(SKBUILD)
    # ensure_all_static(env_model)

    # ensure_static(env_model)
    # ensure_static(crccosy)
    # ensure_static(yaml-cpp)
    # ensure_static(spdlog::spdlog)
    # ensure_static(pugixml-static)
    # ensure_static(protobuf::libprotobuf)

    set_target_properties(s11n PROPERTIES
        VISIBILITY_INLINES_HIDDEN ON
        CXX_VISIBILITY_PRESET hidden
        )
endif()