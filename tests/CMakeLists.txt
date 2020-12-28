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

# Tests
add_executable(env_model_test ${ENV_MODEL_TEST_SRC_FILES})
target_compile_features(env_model_test
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
target_include_directories(env_model_test PUBLIC ${LIB_CR_ENV_MODEL_SRC})
target_link_libraries(env_model_test PUBLIC gtest_main env_model)

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