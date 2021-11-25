include(FetchContent)
include(GNUInstallDirs)

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



# Add yaml-cpp (needs cmake arguments therefore FetchContent_MakeAvailable does not work)
message(STATUS "yaml-cpp - using bundled version")
include(ExternalProject)

ExternalProject_Add(
        yaml_cpp_bundled
        URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.tar.gz
        INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
        BUILD_BYPRODUCTS ${CMAKE_INSTALL_PREFIX}/lib/libyaml-cpp.so
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DBUILD_SHARED_LIBS=ON -DYAML_CPP_BUILD_TESTS=OFF
)
add_library(yaml-cpp SHARED IMPORTED GLOBAL)
add_dependencies(yaml-cpp yaml_cpp_bundled)
set_target_properties(
        yaml-cpp
        PROPERTIES "IMPORTED_LOCATION" "${CMAKE_INSTALL_PREFIX}/lib/libyaml-cpp.so"
)
# create directory otherwise cmake will complain during generate step since this is only generated during make
file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/include")

target_include_directories(yaml-cpp SYSTEM INTERFACE "${CMAKE_INSTALL_PREFIX}/include")
