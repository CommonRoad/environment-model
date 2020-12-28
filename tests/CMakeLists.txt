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
target_compile_definitions(env_model_test PRIVATE)
target_include_directories(env_model_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(env_model_test PUBLIC env_model gtest)

include(GoogleTest)
# queries the compiled executable for tests, this requires the executable to be runnable. if you are
# cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
gtest_discover_tests(
        env_model_test TEST_PREFIX "./"
        # increase the discovery timeout for `env_model_test --gtest_list_tests`
        DISCOVERY_TIMEOUT 60
)


## Code coverage
#set(COVERAGE_GCOVR_EXCLUDES
#        ../src/interfaces/commonroad/pugi_xml/*
#        ../tests/*
#        ../external/*
#        ../docs/*
#        ../cmake/*
#        )
#add_executable(env_model_cov ${UNIT_TEST_FILES} ${LIB_CR_ENV_MODEL_SRC})
#target_link_libraries(env_model_cov gtest_main env_model)
#include(../cmake/CodeCoverage.cmake )
#setup_target_for_coverage_gcovr_html(NAME ${PROJECT_NAME}_coverage
#        EXECUTABLE env_model_cov
#        DEPENDENCIES env_model_cov)
#
#
#
#
# https://dr-kino.github.io/2019/12/22/test-coverage-using-gtest-gcov-and-lcov/