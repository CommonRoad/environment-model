cmake_minimum_required(VERSION 3.20..3.24)
project(EnvironmentModel
    LANGUAGES C CXX
    # TODO: Find an automatic way to sync the version with setuptools_scm/Git tags
    VERSION 2022.0
    HOMEPAGE_URL "https://commonroad.in.tum.de/"
    DESCRIPTION "Classes and methods to represent the CommonRoad format in C++17")

# Adapted from Eigen3 - snippet to get a value for PROJECT_IS_TOP_LEVEL
# on CMake versions before v3.21.0
if(CMAKE_VERSION VERSION_LESS 3.21.0)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL ON)
  else()
    set(PROJECT_IS_TOP_LEVEL OFF)
  endif()
  set(${PROJECT_NAME}_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif()


set(CMAKE_COLOR_DIAGNOSTICS ON)

if(SKBUILD)
        # Scikit-Build does not add your site-packages to the search path
        # automatically, so we need to add it _or_ the pybind11 specific directory
        # here.
        execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c
                "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE _tmp_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE)
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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX("filesystem" COMPILER_SUPPORTS_CXX_FILESYSTEM)
if(NOT COMPILER_SUPPORTS_CXX_FILESYSTEM)
    message(FATAL_ERROR "Your C++ standard library is missing the <filesystem> header "
        "which is required to build the Environment Model. Please try to install a newer "
        "C++ standard library if possible and try again.\n"
        "Remember to start over with a fresh build directory, for example by invoking CMake "
        "with the \"--fresh\" flag.")
endif()

# Some extra debugging for project developers - safe to disable, but read on why they might be useful:

# By enabling CMAKE_LINK_LIBRARIES_ONLY_TARGETS, CMake will report errors whenever
# a name that does not refer to a target known to CMake is passed to
# target_link_libraries and friends.
# We use targets for all dependencies in this project, so if a non-target name is
# passed it usually indicates a bug (usually a typo or a missing find_package call)
# in the CMake configuration.
# Example:
#   target_link_libraries(example_library PUBLIC spdlog)
# This is incorrect as spdlog is not always a target name provided by the spdlog project - it might
# exist in the project itself when it is included via FetchContent, but not if we use a system-provided
# spdlog version.
# Normally, CMake would go on to add "spdlog" as a literal library to the linker command line (e.g. -lspdlog),
# which is not what we want as it might not correct to the correct spdlog library *and* it does not ensure
# the usage requirements (include directories, definitions, other compiler options) are correctly added
# to the compiler command line for example_library.
#
# But since we have CMAKE_LINK_LIBRARIES_ONLY_TARGETS enabled, CMake will instead print an error like this:
# CMake Error at CMakeLists.txt:123456 (target_link_libraries):
#   Target "example_library" has LINK_LIBRARIES_ONLY_TARGETS enabled, but it
#   links to:
#
#     spdlog
#
#   which is not a target.  Possible reasons include:
#
#     * There is a typo in the target name.
#     * A find_package call is missing for an IMPORTED target.
#     * An ALIAS target is missing.

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

# Compile command database is required for Clang-assisted parsing in Doxygen
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

option(BUILD_SHARED_LIBS "Build environment model as a shared library" ON)

option(ENV_MODEL_BUILD_DOXYGEN "Build doxygen" OFF)
option(ENV_MODEL_BUILD_TESTS "Build tests" ON)
# TODO Eigen CMake has a nice approach to setting per-project BUILD_SHARED_LIBS
# option(ENV_MODEL_BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(ENV_MODEL_BUILD_CODE_COVERAGE "Build code coverage" OFF)
option(ENV_MODEL_BUILD_EXECUTABLE "Build executable" ON)

# TODO: Move these
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

# This is a helper script that will automatically add a .gitignore file to the
# binary directory (build directory) so you don't have to do add every build folder
# to your .gitignore.
include(extras/GitIgnoreBinaryDir OPTIONAL)

if(DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external)

include(ExternalBoost)

include(ExternalProtobuf)

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

    find_package(Eigen3 3.3.7 REQUIRED)
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