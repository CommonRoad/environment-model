set(UNIT_TEST_FILES
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

# https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html

add_executable(unit_tests ${UNIT_TEST_FILES})
target_compile_features(unit_tests
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET unit_tests PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(unit_tests PUBLIC ${LIB_CR_ENV_MODEL_SRC})
target_link_libraries(unit_tests PUBLIC gtest_main env_model)

# Code coverage
if (CMAKE_BUILD_TYPE MATCHES Debug)
    if (CodeCoverage MATCHES ON)
        include(../cmake/CodeCoverage.cmake )
        setup_target_for_coverage(${PROJECT_NAME}_coverage poptests coverage)
        setup_target_for_coverage_cobertura(${PROJECT_NAME}_cobertura poptests coverage --gtest_output=xml:coverage.junit.xml)

        set(COVERAGE_GCOVR_EXCLUDES
                ../src/interfaces/commonroad/pugi_xml/*
                ../src/tests/*
                ../src/external/*
                ../src/docs/*
                ../src/cmake/*
                )
        add_executable(poptests ${UNIT_TEST_FILES} ${LIB_CR_ENV_MODEL_SRC})
        target_link_libraries(poptests gtest_main env_model)
    endif()
endif()

