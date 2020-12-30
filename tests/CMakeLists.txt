set(ENV_MODEL_TEST_SRC_FILES
        envModelTests/allTests.cpp
        envModelTests/roadNetwork/test_road_network.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet_operations.cpp
        envModelTests/roadNetwork/lanelet/test_lane.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet.cpp
        envModelTests/obstacle/test_obstacle.cpp
        envModelTests/obstacle/test_state.cpp
        envModelTests/obstacle/test_obstacle_operations.cpp
        envModelTests/geometry/test_rectangle.cpp
        envModelTests/geometry/test_circle.cpp
        envModelTests/geometry/test_geometric_operations.cpp
        envModelTests/interfaces/test_command_line_input.cpp
        envModelTests/interfaces/test_reading_commonroad.cpp)

add_subdirectory("${PROJECT_SOURCE_DIR}/external/googletest" "external/googletest")

mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

add_executable(env_model_test ${ENV_MODEL_TEST_SRC_FILES})
target_compile_features(env_model_example_node
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model_example_node PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET env_model_example_node PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
target_link_libraries(env_model_test PUBLIC env_model gtest)


include(GoogleTest)
# queries the compiled executable for tests, this requires the executable to be runnable. if you are
# cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
gtest_discover_tests(
        env_model_test TEST_PREFIX "./"
        # increase the discovery timeout for `env_model_test --gtest_list_tests`
        DISCOVERY_TIMEOUT 60
)

# Code coverage
if (CMAKE_BUILD_TYPE MATCHES Debug)
    if (CodeCoverage MATCHES ON)
        add_executable(env_model_test_coverage ${ENV_MODEL_SRC_FILES} ${ENV_MODEL_TEST_SRC_FILES})
        target_include_directories(env_model_test_coverage PUBLIC "./../src/" ${LIB_CRCCOSY_INCLUDE})
        target_link_libraries(env_model_test_coverage PUBLIC
                env_model
                ${CRCCOSY_LIBRARY}
                gtest
                Boost::filesystem
                Boost::program_options
                OpenMP::OpenMP_CXX
                Eigen3::Eigen)

        include(CodeCoverage)
        append_coverage_compiler_flags()
         setup_target_for_coverage_gcovr_html(
             NAME env_model_coverage                                          # New target name
             EXECUTABLE env_model_test_coverage                               # Executable in PROJECT_BINARY_DIR
             DEPENDENCIES env_model_test_coverage                             # Dependencies to build first
             BASE_DIRECTORY "../"                                             # Base directory for report
                                                                              #  (defaults to PROJECT_SOURCE_DIR)
             EXCLUDE                                                          # Patterns to exclude (can be relative
                 "src/interfaces/commonroad/pugi_xml/pugixml.hpp"             #  to BASE_DIRECTORY, with CMake 3.4+)
                 "src/interfaces/commonroad/pugi_xml/pugixml.cpp"
                 "ci/"
                 "docs/"
                 "external/"
         )
    endif()
endif()
