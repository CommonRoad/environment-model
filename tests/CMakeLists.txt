set(ENV_MODEL_TEST_SRC_FILES
        commonroad_cpp_tests/interfaces/utility_functions.cpp
        allTests.cpp
        commonroad_cpp_tests/test_world.cpp
        commonroad_cpp_tests/test_commonroad_container.cpp
        commonroad_cpp_tests/roadNetwork/test_road_network.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lanelet_operations.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lane.cpp
        commonroad_cpp_tests/roadNetwork/lanelet/test_lanelet.cpp
        commonroad_cpp_tests/obstacle/test_obstacle.cpp
        commonroad_cpp_tests/obstacle/test_state.cpp
        commonroad_cpp_tests/obstacle/test_obstacle_operations.cpp
        commonroad_cpp_tests/geometry/test_rectangle.cpp
        commonroad_cpp_tests/geometry/test_circle.cpp
        commonroad_cpp_tests/geometry/test_geometric_operations.cpp
        commonroad_cpp_tests/interfaces/test_interfaces.cpp
        commonroad_cpp_tests/auxiliaryDefs/test_timer.cpp
        commonroad_cpp_tests/predicates/predicate_manager_test.cpp
        commonroad_cpp_tests/predicates/utils_predicate_test.cpp
        commonroad_cpp_tests/predicates/braking/test_safe_distance_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_unnecessary_braking_predicate.cpp
        commonroad_cpp_tests/predicates/braking/test_brakes_stronger_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_in_front_of_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_in_same_lane_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_in_single_lane_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_left_of_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_orientation_towards_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_lane_based_orientation_similar_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_in_congestion_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_in_slow_moving_traffic_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_in_queue_of_vehicles_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_makes_u_turn_predicate.cpp
        commonroad_cpp_tests/predicates/general/test_interstate_broad_enough.cpp
        commonroad_cpp_tests/predicates/position/test_in_intersection_main_area_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_stop_line_in_front_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_passes_stop_line_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_on_access_ramp_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_on_exit_ramp_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_on_shoulder_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_on_urban_road_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_in_leftmost_lane_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_in_rightmost_lane_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_drives_leftmost_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_drives_rightmost_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_on_main_carriage_way_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_right_of_broad_lane_marking_predicate.cpp
        commonroad_cpp_tests/predicates/position/test_left_of_broad_lane_marking_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_red_right_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_red_left_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_red_straight_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_red_traffic_light_predicate.cpp
        commonroad_cpp_tests/predicates/regulatory/test_at_stop_sign_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_keeps_lane_speed_limit_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_required_speed_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_preserves_traffic_flow_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_slow_leading_vehicle_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_in_standstill_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_exist_standing_leading_vehicle_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_drives_faster_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_drives_with_slightly_higher_speed_predicate.cpp
        commonroad_cpp_tests/predicates/velocity/test_reverses_predicate.cpp
        )


# Create executable for test
if (ENV_MODEL_BUILD_TESTS)
    add_executable(env_model_test ${ENV_MODEL_TEST_SRC_FILES} commonroad_cpp_tests/predicates/predicate_manager_test.cpp commonroad_cpp_tests/predicates/predicate_manager_test.h commonroad_cpp_tests/test_commonroad_container.cpp commonroad_cpp_tests/test_commonroad_container.h commonroad_cpp_tests/auxiliaryDefs/test_timer.cpp commonroad_cpp_tests/auxiliaryDefs/test_timer.h commonroad_cpp_tests/roadNetwork/regulartoryElements/test_regulatory_elements_utils.cpp commonroad_cpp_tests/roadNetwork/regulartoryElements/test_regulatory_elements_utils.h)

    target_compile_features(env_model_test
            PUBLIC cxx_lambdas cxx_auto_type
            PRIVATE cxx_lambdas cxx_auto_type)

    set_property(TARGET env_model_test PROPERTY POSITION_INDEPENDENT_CODE ON)

    target_link_libraries(env_model_test
            PRIVATE
            env_model
            gtest
            Boost::filesystem
            OpenMP::OpenMP_CXX
            spdlog::spdlog
            )

    include(GoogleTest)
    # queries the compiled executable for tests, this requires the executable to be runnable. if you are
    # cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
    gtest_discover_tests(
            env_model_test TEST_PREFIX "./"
            # increase the discovery timeout for `env_model_test --gtest_list_tests`
            DISCOVERY_TIMEOUT 60
    )
endif ()


# Code coverage
if (CMAKE_BUILD_TYPE MATCHES Debug)
    if (ENV_MODEL_BUILD_CODE_COVERAGE)
        add_executable(env_model_test_coverage ${ENV_MODEL_SRC_FILES} ${ENV_MODEL_TEST_SRC_FILES})
        target_include_directories(env_model_test_coverage PUBLIC "./../src/" ${LIB_CRCCOSY_INCLUDE})
        target_link_libraries(env_model_test_coverage PUBLIC
                env_model
                DrivabilityChecker::crccosy
                gtest
                Boost::filesystem
                Boost::program_options
                OpenMP::OpenMP_CXX
                Eigen3::Eigen
                spdlog::spdlog
                )

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