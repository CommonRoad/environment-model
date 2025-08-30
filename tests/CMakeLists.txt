set(ENV_MODEL_TEST_SRC_FILES
        commonroad_cpp_tests/interfaces/utility_functions.cpp
        allTests.cpp
        commonroad_cpp_tests/test_world.cpp
        commonroad_cpp_tests/test_planning_problem.cpp
        commonroad_cpp_tests/roadNetwork/test_road_network.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lanelet_operations.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lane.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lanelet.cpp
        commonroad_cpp_tests/roadNetwork/regulatoryElements/test_stop_line.cpp
        commonroad_cpp_tests/roadNetwork/regulatoryElements/test_traffic_sign.cpp
        commonroad_cpp_tests/roadNetwork/regulatoryElements/test_traffic_sign_element.cpp
        commonroad_cpp_tests/roadNetwork/regulatoryElements/test_traffic_light.cpp
        commonroad_cpp_tests/roadNetwork/regulatoryElements/test_regulatory_elements_utils.cpp
        commonroad_cpp_tests/roadNetwork/intersection/test_incoming.cpp
        commonroad_cpp_tests/roadNetwork/intersection/test_intersection.cpp
        commonroad_cpp_tests/obstacle/test_obstacle.cpp
        commonroad_cpp_tests/obstacle/test_state.cpp
        commonroad_cpp_tests/obstacle/test_occupancy.cpp
        commonroad_cpp_tests/obstacle/test_signal_state.cpp
        commonroad_cpp_tests/obstacle/test_obstacle_operations.cpp
        commonroad_cpp_tests/geometry/test_rectangle.cpp
        commonroad_cpp_tests/geometry/test_circle.cpp
        commonroad_cpp_tests/geometry/test_polygon.cpp
        commonroad_cpp_tests/geometry/test_shape_group.cpp
        commonroad_cpp_tests/geometry/test_geometric_operations.cpp
        commonroad_cpp_tests/interfaces/test_interfaces.cpp
        commonroad_cpp_tests/auxiliaryDefs/test_timer.cpp

        commonroad_cpp_tests/predicates/predicate_config_test.cpp
        commonroad_cpp_tests/predicates/predicate_manager_test.cpp
        commonroad_cpp_tests/predicates/utils_predicate_test.cpp

        commonroad_cpp_tests/predicates/braking/test_keeps_safe_distance_prec_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_brakes_stronger_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_causes_braking_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_braking_at_intersection_possible_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_decelerates_predicate.cpp

        commonroad_cpp_tests/predicates/misc/test_low_computation_time_test_predicate.cpp
        commonroad_cpp_tests/predicates/misc/test_high_computation_time_test_predicate.cpp

        commonroad_cpp_tests/predicates/general/test_orientation_towards_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_signal_set_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_makes_u_turn_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_is_of_type_predicate.cpp

        commonroad_cpp_tests/predicates/intersection/test_at_intersection_type_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_approach_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_in_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_at_same_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_on_single_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_in_intersection_conflict_area_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_close_to_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_unobstructed_intersection_view_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_on_incoming_left_of_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_on_incoming_of_intersection_predicate.cpp
        commonroad_cpp_tests/predicates/intersection/test_on_oncoming_of_predicate.cpp

        commonroad_cpp_tests/predicates/lane/test_overtaking_allowed_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_interstate_broad_enough_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_narrow_road_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_lane_based_orientation_similar_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_two_or_more_lanes_for_one_dir_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_adjacent_lanelet_of_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_lanelet_with_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_lanelet_with_successor_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_road_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_similar_oriented_lanelet_with_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_similar_oriented_lanelet_without_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_in_outermost_lane_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_close_to_lane_border_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_completely_on_lanelet_type_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_in_same_lane_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_in_single_lane_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_parallel_of_lane_marking_of_type_on_side_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_in_neighboring_lane_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_neighboring_lane_opp_driving_dir_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_lane_based_orientation_side_predicate.cpp
        commonroad_cpp_tests/predicates/lane/test_on_same_road_predicate.cpp

        commonroad_cpp_tests/predicates/position/test_in_front_of_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_left_of_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_right_of_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_lateral_distance_to_obstacle_above_threshold_predicate.cpp

        commonroad_cpp_tests/predicates/regulatory/test_at_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_traffic_sign_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_stop_line_in_front_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_close_to_stop_line_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_relevant_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_has_priority_predicate.cpp

        commonroad_cpp_tests/predicates/velocity/test_preserves_traffic_flow_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_slow_other_vehicle_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_lane_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_min_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_in_standstill_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_drives_faster_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_reverses_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_type_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_fov_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_braking_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_velocity_below_threshold_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_drives_with_slightly_higher_speed_predicate.cpp
)


# Create executable for test
if (ENV_MODEL_BUILD_TESTS)
    add_executable(env_model_test ${ENV_MODEL_TEST_SRC_FILES})

    target_compile_features(env_model_test
            PUBLIC cxx_lambdas cxx_auto_type
            PRIVATE cxx_lambdas cxx_auto_type)

    set_property(TARGET env_model_test PROPERTY POSITION_INDEPENDENT_CODE ON)

    target_link_libraries(env_model_test
            PRIVATE
            env_model
            GTest::gtest
            OpenMP::OpenMP_CXX
            spdlog::spdlog
            )

    if(APPLE)
        # Required to find library -lomp on mac
        # MAC_LIBOMP_PATH defined in root CMakeLists.txt
        target_link_directories(env_model_test PUBLIC "${MAC_LIBOMP_PATH}/lib")
    endif()

    target_precompile_headers(env_model_test
            PRIVATE
            <gtest/gtest.h>)

    include(GoogleTest)
    # queries the compiled executable for tests, this requires the executable to be runnable. if you are
    # cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
    gtest_discover_tests(
            env_model_test
            # TEST_PREFIX "./"
            # increase the discovery timeout for `env_model_test --gtest_list_tests`
            DISCOVERY_TIMEOUT 60
            WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
            XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports
    )
endif ()


# Code coverage
# NOTE: The CodeCoverage module will already warn if the build type is not Debug
if (ENV_MODEL_BUILD_CODE_COVERAGE)
    include(../cmake/CodeCoverage.cmake)

    # Add definition for toolchain::gcov
    include(toolchain/ToolchainLibraries)

    # Unfortunately, append_coverage_compiler_flags_to_target is currently broken,
    # so we simply add the required flags ourselves

    set(COVERAGE_FLAGS -g -fprofile-arcs -ftest-coverage -fprofile-abs-path)

    set_property(SOURCE allTests.cpp PROPERTY COMPILE_DEFINITIONS CODE_COVERAGE)

    foreach(target env_model_core env_model_predicates env_model_test)
        # append_coverage_compiler_flags_to_target(${target})
        target_compile_options(${target} PRIVATE ${COVERAGE_FLAGS})
        target_link_libraries(${target} PRIVATE toolchain::gcov)
    endforeach()

    setup_target_for_coverage_gcovr_html(
            NAME env_model_coverage
            EXECUTABLE env_model_test
            EXCLUDES ${FETCHCONTENT_BASE_DIR}
    )
endif()
