include(FetchContent)
include(GNUInstallDirs)
include(ExternalProject)

# pugixml
FetchContent_Declare(
  external_pugixml

  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG        v1.11.4
  GIT_PROGRESS   true
)

# pybind11
FetchContent_Declare(
  external_pybind11

  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.8.1
  GIT_PROGRESS   true

)

# GoogleTest
FetchContent_Declare(
  external_googletest

  GIT_REPOSITORY https://github.com/google/googletest.git
  # Commit for v1.11.0
  GIT_TAG        e2239ee6043f73722e7aa812a459f54a28552929
  GIT_PROGRESS   true
)

# YAML-CPP
FetchContent_Declare(
        external_yaml_cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.6.3
)
#
## Curvilinear Coordinate System
#FetchContent_Declare(
#        external_crccosy
#        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
#        GIT_TAG "development"
#)

# Stage 2: Run FetchContent_MakeAvailable
# CMake recommends running FetchContent_Declare for all third-party projects first
# and finally running FetchContent_MakeAvailable in the end
# See https://cmake.org/cmake/help/v3.22/module/FetchContent.html#command:fetchcontent_makeavailable

FetchContent_MakeAvailable(external_pugixml)

set(PYBIND11_CPP_STANDARD -std=c++17)
FetchContent_MakeAvailable(external_pybind11)

FetchContent_MakeAvailable(external_googletest)

# Add google-test
mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

# Install rule for pugixml headers
install(
  DIRECTORY ${external_pugixml_SOURCE_DIR}/src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
  )


set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(YAML_CPP_BUILD_TESTS OFF)
set(YAML_CPP_BUILD_TOOLS OFF)
FetchContent_MakeAvailable(external_yaml_cpp)
set_property(TARGET yaml-cpp
        PROPERTY POSITION_INDEPENDENT_CODE ON)
# Install rule for yaml-cpp headers
install(
        DIRECTORY ${external_yaml_cpp_SOURCE_DIR}/src/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp"
)

#DrivabilityChecker (includes only curvilinear coordinate system)
#set(ADD_MODULE_COLLISION OFF)
#set(BUILD_LIBCCD OFF)
#set(BUILD_FCL OFF)
#set(BUILD_S11N OFF)
#set(BUILD_BOX2D OFF)
#set(ADD_PYTHON_BINDINGS OFF)
#FetchContent_MakeAvailable(external_crccosy)
#install(
#        DIRECTORY ${external_crccosy_SOURCE_DIR}/cpp/geometry/
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#        FILES_MATCHING PATTERN "*.h"
#)
#message(STATUS "DrivabilityChecker - using bundled version")
ExternalProject_Add(
        DrivabilityChecker_bundled
        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
        GIT_TAG "development"
        INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
        BUILD_BYPRODUCTS ${CMAKE_INSTALL_PREFIX}/lib/libcrccosy.a
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DADD_MODULE_COLLISION=OFF -DBUILD_LIBCCD=OFF -DBUILD_FCL=OFF -DBUILD_S11N=OFF -DBUILD_BOX2D=OFF
)

add_library(DrivabilityChecker::crccosy SHARED IMPORTED GLOBAL)
add_dependencies(DrivabilityChecker::crccosy DrivabilityChecker_bundled)
set_target_properties(
        DrivabilityChecker::crccosy
        PROPERTIES "IMPORTED_LOCATION" "${CMAKE_INSTALL_PREFIX}/lib/libcrccosy.a"
)
# create directory otherwise cmake will complain during generate step since this is only generated during make
file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/include")
target_include_directories(DrivabilityChecker::crccosy SYSTEM INTERFACE "${CMAKE_INSTALL_PREFIX}/include")


