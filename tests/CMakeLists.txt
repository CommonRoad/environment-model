add_executable(unit_tests
        envModelTests/allTests.cpp
        envModelTests/roadNetwork/test_road_network.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet_operations.cpp
        envModelTests/roadNetwork/lanelet/test_lane.cpp
        envModelTests/roadNetwork/lanelet/test_lanelet.cpp
        envModelTests/obstacle/test_obstacle.cpp
        envModelTests/obstacle/test_state.cpp
        envModelTests/obstacle/test_obstacle_operations.cpp)
target_compile_features(unit_tests
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET unit_tests PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(unit_tests PUBLIC ${LIB_CR_ENV_MODEL_SRC})
target_link_libraries(unit_tests PUBLIC gtest_main env_model)

