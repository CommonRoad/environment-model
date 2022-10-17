set(ENV_MODEL_SRC_FILES
        commonroad_cpp/auxiliaryDefs/regulatory_elements.cpp
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
        commonroad_cpp/interfaces/commonroad/input_utils.cpp
        commonroad_cpp/world.cpp
        commonroad_cpp/commonroad_container.cpp
        commonroad_cpp/auxiliaryDefs/timer.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_graph.cpp
        commonroad_cpp/interfaces/commonroad/protobuf_reader.cpp)

set(ENV_MODEL_PREDICATES_SRC_FILES
        commonroad_cpp/predicates/commonroad_predicate.cpp
        commonroad_cpp/predicates/braking/safe_distance_predicate.cpp
        commonroad_cpp/predicates/braking/unnecessary_braking_predicate.cpp
        commonroad_cpp/predicates/braking/brakes_stronger_predicate.cpp
        commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.cpp
        commonroad_cpp/predicates/braking/braking_with_acceleration_possible_at_intersection_predicate.cpp
        commonroad_cpp/predicates/position/in_front_of_predicate.cpp
        commonroad_cpp/predicates/position/in_same_lane_predicate.cpp
        commonroad_cpp/predicates/position/in_single_lane_predicate.cpp
        commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.cpp
        commonroad_cpp/predicates/position/right_of_broad_lane_marking_predicate.cpp
        commonroad_cpp/predicates/position/left_of_broad_lane_marking_predicate.cpp
        commonroad_cpp/predicates/position/left_of_predicate.cpp
        commonroad_cpp/predicates/position/main_carriageway_right_lane_predicate.cpp
        commonroad_cpp/predicates/position/on_lanelet_with_type_predicate.cpp
        commonroad_cpp/predicates/position/on_similar_oriented_lanelet_with_type_predicate.cpp
        commonroad_cpp/predicates/position/on_similar_oriented_lanelet_without_type_predicate.cpp
        commonroad_cpp/predicates/position/in_leftmost_lane_predicate.cpp
        commonroad_cpp/predicates/position/in_rightmost_lane_predicate.cpp
        commonroad_cpp/predicates/position/drives_leftmost_predicate.cpp
        commonroad_cpp/predicates/position/drives_rightmost_predicate.cpp
        commonroad_cpp/predicates/position/unobstructed_intersection_view_predicate.cpp
        commonroad_cpp/predicates/position/on_incoming_left_of_predicate.cpp
        commonroad_cpp/predicates/position/on_oncoming_of_predicate.cpp
        commonroad_cpp/predicates/position/in_intersection_conflict_area_predicate.cpp
        commonroad_cpp/predicates/general/orientation_towards_predicate.cpp
        commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.cpp
        commonroad_cpp/predicates/general/in_congestion_predicate.cpp
        commonroad_cpp/predicates/general/in_slow_moving_traffic_predicate.cpp
        commonroad_cpp/predicates/general/in_queue_of_vehicles_predicate.cpp
        commonroad_cpp/predicates/general/makes_u_turn_predicate.cpp
        commonroad_cpp/predicates/general/interstate_broad_enough_predicate.cpp
        commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.cpp
        commonroad_cpp/predicates/velocity/slow_leading_vehicle_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_lane_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/required_speed_predicate.cpp
        commonroad_cpp/predicates/velocity/in_standstill_predicate.cpp
        commonroad_cpp/predicates/velocity/exist_standing_leading_vehicle.cpp
        commonroad_cpp/predicates/velocity/drives_faster_predicate.cpp
        commonroad_cpp/predicates/velocity/reverses_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_type_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_braking_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_traffic_sign_predicate.cpp
        commonroad_cpp/predicates/regulatory/relevant_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/same_priority_predicate.cpp
        commonroad_cpp/predicates/regulatory/has_priority_predicate.cpp
        commonroad_cpp/predicates/predicate_config.cpp)

add_library(env_model_core OBJECT ${ENV_MODEL_SRC_FILES})

target_compile_features(env_model_core
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET env_model_core PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(env_model_core
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

# if(pugixml_FOUND)
#         add_library(pugixml ALIAS pugixml::pugixml)
# elseif(SKBUILD)
#         add_library(pugixml ALIAS pugixml::static)
# else()
#         add_library(pugixml ALIAS pugixml::pugixml)
# endif()

# find_package(pugixml)
target_link_libraries(env_model_core
        PRIVATE
        # Not used in any headers
        Eigen3::Eigen
        $<BUILD_INTERFACE:yaml-cpp::yaml-cpp>
        OpenMP::OpenMP_CXX
        # pugixml::pugixml-static
        # pugixml::pugixml
        #pugixml-static
        pugixml::pugixml
        # spdlog::spdlog
        $<BUILD_INTERFACE:spdlog::spdlog>
)

target_link_libraries(env_model_core
        PUBLIC
        crccosy
        range-v3::range-v3
        commonroad_protobuf
        )


add_library(env_model_predicates OBJECT ${ENV_MODEL_PREDICATES_SRC_FILES})

target_include_directories(env_model_predicates
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

target_link_libraries(env_model_predicates
        PUBLIC
        Eigen3::Eigen
        crccosy
        #DrivabilityChecker::crccosy
        range-v3
        env_model_core
        )

set_property(TARGET env_model_predicates PROPERTY POSITION_INDEPENDENT_CODE ON)


# Create environment model library
add_library(env_model)
# if(ENV_MODEL_BUILD_SHARED_LIBS)
#     add_library(env_model SHARED)
# else()
#     add_library(env_model STATIC)
# endif()

target_link_libraries(env_model
        PUBLIC
        env_model_core
        env_model_predicates
        commonroad_protobuf
        )

# add pybind module
if(SKBUILD)
    message(STATUS "The project is built using scikit-build")
endif()
#if(ENV_MODEL_BUILD_PYBIND)
if(SKBUILD)
    add_library(env_model_python OBJECT
            commonroad_cpp/interfaces/pybind/python_interface.cpp
            commonroad_cpp/interfaces/pybind/python_interface.h
            commonroad_cpp/interfaces/pybind/translate_python_types.cpp
            commonroad_cpp/interfaces/pybind/translate_python_types.h)
    target_link_libraries(env_model_python PRIVATE env_model pybind11::module)
    set_property(TARGET env_model_python PROPERTY POSITION_INDEPENDENT_CODE ON)

    set(python_module_name crcpp)
    pybind11_add_module(${python_module_name} MODULE $<TARGET_OBJECTS:env_model_python>)

    target_link_libraries(${python_module_name} PRIVATE env_model)

    install(TARGETS ${python_module_name} env_model
            LIBRARY DESTINATION .
            COMPONENT PythonModules
            )

    add_custom_target(install-python-modules-${python_module_name}
        ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_COMPONENT=PythonModules
        -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
        DEPENDS ${python_module_name}
        )


    # include(utils/CheckRuntimeDependencies)
    # check_python_runtime_dependencies(${python_module_name})

endif(SKBUILD)

# this adds many warnings to the build. They usually help to find some bugs
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(env_model PRIVATE -Wall -Wextra -Wconversion
            -pedantic -Wfatal-errors -Wno-unused-parameter)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(env_model PRIVATE /W3 /WX)
endif()


