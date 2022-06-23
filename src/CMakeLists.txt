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
        commonroad_cpp/commonroad_container.cpp
        commonroad_cpp/predicates/predicate_config.cpp
        commonroad_cpp/auxiliaryDefs/timer.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_graph.cpp
        commonroad_cpp/interfaces/commonroad/protobuf_reader.cpp
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/commonroad.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/dynamic_obstacle.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/environment_obstacle.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/intersection.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/lanelet.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/location.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/obstacle.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/phantom_obstacle.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/planning_problem.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/scenario_tags.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/static_obstacle.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/traffic_light.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/traffic_sign.pb.cc
        commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/util.pb.cc)

# Create environment model library
if(ENV_MODEL_BUILD_SHARED_LIBS)
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
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

target_link_libraries(env_model
        PUBLIC
        Eigen3::Eigen
        DrivabilityChecker::crccosy
        )

target_link_libraries(env_model
        PUBLIC
        Eigen3::Eigen
        DrivabilityChecker::crccosy
        yaml-cpp
        ${PROTOBUF_LIBRARY}
        )


target_link_libraries(env_model
        PRIVATE
        OpenMP::OpenMP_CXX
        pugixml-static
        spdlog::spdlog
        )

# add pybind module
if(SKBUILD)
    message(STATUS "The project is built using scikit-build")
endif()
if(ENV_MODEL_BUILD_PYBIND)
    set(python_module_name crcpp)
    pybind11_add_module(${python_module_name} MODULE
            commonroad_cpp/interfaces/pybind/python_interface.cpp
            commonroad_cpp/interfaces/pybind/python_interface.h
            commonroad_cpp/interfaces/pybind/translate_python_types.cpp
            commonroad_cpp/interfaces/pybind/translate_python_types.h)

    target_link_libraries(${python_module_name} PRIVATE env_model)

    install(TARGETS ${python_module_name} env_model
            LIBRARY DESTINATION .
            )

    set_property(TARGET ${python_module_name} PROPERTY INSTALL_RPATH "$ORIGIN")

endif(ENV_MODEL_BUILD_PYBIND)

# this adds many warnings to the build. They usually help to find some bugs
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(env_model PRIVATE -Wall -Wextra -Wconversion
            -pedantic -Wfatal-errors -Wno-unused-parameter)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(env_model PRIVATE /W3 /WX)
endif()


