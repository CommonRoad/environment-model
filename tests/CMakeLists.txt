add_executable(unit_tests envModelTests/allTests.cpp envModelTests/roadNetwork/test_lanelet.cpp)
target_compile_features(unit_tests
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET unit_tests PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(unit_tests PUBLIC ${LIB_CR_ENV_MODEL_SRC})
target_link_libraries(unit_tests PUBLIC gtest_main env_model)

