protobuf_generate_cpp(
        COMMONROAD_PB_SRCS
        COMMONROAD_PB_HDRS
        commonroad.proto
        dynamic_obstacle.proto
        environment_obstacle.proto
        intersection.proto
        lanelet.proto
        location.proto
        obstacle.proto
        phantom_obstacle.proto
        planning_problem.proto
        scenario_tags.proto
        static_obstacle.proto
        traffic_light.proto
        traffic_sign.proto
        util.proto
        )

add_library(commonroad_protobuf OBJECT
    ${COMMONROAD_PB_SRCS}
    ${COMMONROAD_PB_HDRS}
    )

target_include_directories(commonroad_protobuf
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        )

target_link_libraries(commonroad_protobuf
        PUBLIC
        $<BUILD_INTERFACE:protobuf::libprotobuf>
        )

set_property(TARGET commonroad_protobuf PROPERTY POSITION_INDEPENDENT_CODE ON)