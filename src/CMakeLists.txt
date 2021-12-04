set(ENV_MODEL_SRC_FILES
        commonroad_cpp/auxiliaryDefs/traffic_signs.cpp
        commonroad_cpp/roadNetwork/road_network.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet.cpp
        commonroad_cpp/roadNetwork/lanelet/lane.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_operations.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/stop_line.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.cpp
        commonroad_cpp/roadNetwork/intersection/incoming.cpp
        commonroad_cpp/roadNetwork/intersection/intersection.cpp
        commonroad_cpp/roadNetwork/intersection/intersection_operations.cpp
        commonroad_cpp/obstacle/obstacle.cpp
        commonroad_cpp/obstacle/obstacle_operations.cpp
        commonroad_cpp/obstacle/state.cpp
        commonroad_cpp/geometry/circle.cpp
        commonroad_cpp/geometry/geometric_operations.cpp
        commonroad_cpp/geometry/rectangle.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2018b.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2020a.cpp
        commonroad_cpp/interfaces/commonroad/xml_reader.cpp
        commonroad_cpp/interfaces/standalone/command_line_input.cpp
        commonroad_cpp/world.cpp
        commonroad_cpp/predicates/predicate_manager.cpp
        commonroad_cpp/predicates/commonroad_predicate.cpp
        commonroad_cpp/predicates/braking/safe_distance_predicate.cpp
        commonroad_cpp/predicates/braking/unnecessary_braking_predicate.cpp
        commonroad_cpp/predicates/position/in_front_of_predicate.cpp
        commonroad_cpp/predicates/position/in_same_lane_predicate.cpp
        commonroad_cpp/predicates/position/in_single_lane_predicate.cpp
        commonroad_cpp/predicates/position/stop_line_in_front_predicate.cpp
        commonroad_cpp/predicates/position/passes_stop_line_predicate.cpp
        commonroad_cpp/predicates/general/orientation_towards_predicate.cpp
        commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.cpp
        commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.cpp
        commonroad_cpp/predicates/velocity/slow_leading_vehicle_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_lane_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/required_speed_predicate.cpp
        commonroad_cpp/predicates/velocity/in_standstill_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_red_right_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_red_left_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_red_straight_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_red_traffic_light_predicate.cpp
        commonroad_cpp/predicates/position/in_intersection_main_area_predicate.cpp
        commonroad_cpp/commonroad_container.cpp
        commonroad_cpp/predicates/predicate_config.cpp
        commonroad_cpp/auxiliaryDefs/timer.cpp
        )

# Create environment model library
if(ENVMODEL_BUILD_SHARED_LIBS)
add_library(env_model SHARED ${ENV_MODEL_SRC_FILES})
else()
    add_library(env_model STATIC ${ENV_MODEL_SRC_FILES})
endif()

target_compile_features(env_model
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET env_model PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(env_model
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

target_link_libraries(env_model
    PUBLIC
        Eigen3::Eigen
        DrivabilityChecker::crccosy
        yaml-cpp
    )

target_link_libraries(env_model
    PRIVATE
        Boost::program_options
        Boost::filesystem
        OpenMP::OpenMP_CXX
        pugixml-static
        spdlog::spdlog
    )

if(ENV_MODEL_BUILD_EXECUTABLE)
    # Create environment model test node
    add_executable(env_model_example_node commonroad_cpp/interfaces/standalone/main.cpp)

    target_link_libraries(env_model_example_node
        PRIVATE
            env_model
            Boost::program_options
            spdlog::spdlog
            yaml-cpp
            pthread dl
        )
endif(ENV_MODEL_BUILD_EXECUTABLE)

# add pybind module
if(ENV_MODEL_BUILD_PYBIND)
    pybind11_add_module(cpp_env_model
            commonroad_cpp/interfaces/pybind/python_interface.cpp
            commonroad_cpp/interfaces/pybind/python_interface.h
            commonroad_cpp/interfaces/pybind/translate_python_types.cpp
            commonroad_cpp/interfaces/pybind/translate_python_types.h)

    target_link_libraries(cpp_env_model PRIVATE env_model)

    # $ORIGIN is a special value: It represents the location of the shared object
    # being loaded itself. By setting the RPATH of the installed Python module
    # to $ORIGIN, the loader will automatically look for shared libraries in the
    # same directory as the module file itself.
    #
    # This is required here so that the loader finds e.g. libcrcc.so which is
    # installed next to the Python module, even if the Python module got moved
    # during installation.

    set_property(TARGET cpp_env_model PROPERTY INSTALL_RPATH "$ORIGIN")
    set_property(TARGET yaml-cpp PROPERTY INSTALL_RPATH "$ORIGIN")

endif(ENV_MODEL_BUILD_PYBIND)

# this adds many warnings to the build. They usually help to find some bugs
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    if(ENV_MODEL_BUILD_EXECUTABLE)
        target_compile_options(env_model_example_node PRIVATE -Wall -Wextra -Wconversion -pedantic
                -Wfatal-errors)
    endif(ENV_MODEL_BUILD_EXECUTABLE)
    target_compile_options(env_model PRIVATE -Wall -Wextra -Wconversion
            -pedantic -Wfatal-errors)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(env_model PRIVATE /W3 /WX)
    if(ENV_MODEL_BUILD_EXECUTABLE)
        target_compile_options(env_model_example_node PRIVATE /W3 /WX)
    endif(ENV_MODEL_BUILD_EXECUTABLE)
endif()


