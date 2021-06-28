set(ENV_MODEL_TEST_SRC_FILES
        envModelTests/interfaces/utility_functions.cpp
        allTests.cpp
        envModelTests/roadNetwork/test_road_network.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet_operations.cpp
        envModelTests/roadNetwork/lanelet/test_lane.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet.cpp
        envModelTests/obstacle/test_obstacle.cpp
        envModelTests/obstacle/test_state.cpp
        envModelTests/obstacle/test_obstacle_operations.cpp
        envModelTests/geometry/test_rectangle.cpp
        envModelTests/geometry/test_geometric_operations.cpp
        envModelTests/interfaces/test_interfaces.cpp
        envModelTests/predicates/braking/test_safe_distance_predicate.cpp
        envModelTests/predicates/braking/test_unnecessary_braking_predicate.cpp
        envModelTests/predicates/position/test_in_front_of_predicate.cpp
        envModelTests/predicates/position/test_in_same_lane_predicate.cpp
        envModelTests/predicates/general/test_cut_in_predicate.cpp
        envModelTests/predicates/position/test_in_intersection_main_area_predicate.cpp
        envModelTests/predicates/position/test_stop_line_in_front_predicate.cpp
        envModelTests/predicates/position/test_passes_stop_line_predicate.cpp
        envModelTests/predicates/regulatory/test_at_red_right_traffic_light_predicate.cpp
        envModelTests/predicates/regulatory/test_at_red_left_traffic_light_predicate.cpp
        envModelTests/predicates/regulatory/test_at_red_straight_traffic_light_predicate.cpp
        envModelTests/predicates/regulatory/test_at_red_traffic_light_predicate.cpp
        envModelTests/predicates/velocity/test_keeps_lane_speed_limit_predicate.cpp
        envModelTests/predicates/velocity/test_required_speed_predicate.cpp
        envModelTests/predicates/velocity/test_preserves_traffic_flow_predicate.cpp
        envModelTests/predicates/velocity/test_slow_leading_vehicle_predicate.cpp
        envModelTests/predicates/velocity/test_in_standstill_predicate.cpp
        envModelTests/predicates/utils_predicate_test.cpp
        )

# Add google-test
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

# Create executable for test
add_executable(env_model_test ${ENV_MODEL_TEST_SRC_FILES})
target_compile_features(env_model_test
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model_test PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(env_model_test PRIVATE env_model gtest Boost::filesystem)

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
                DrivabilityChecker::crccosy
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
