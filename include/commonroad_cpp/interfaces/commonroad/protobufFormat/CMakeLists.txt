set(COMMONROAD_PROTOS
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

# CMP0116 (3.20) - Ninja generators transform DEPFILEs from add_custom_command().
if(POLICY CMP0116)
        cmake_policy(SET CMP0116 NEW)
endif()

add_library(commonroad_protobuf)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    # Set up header file set
    target_sources(commonroad_protobuf
            INTERFACE
            FILE_SET commonroad_protobuf_headers
            TYPE HEADERS
            BASE_DIRS ${PROJECT_BINARY_DIR}/include
    )
endif()

foreach(proto ${COMMONROAD_PROTOS})
        set(proto_orig "${proto}")
        set(proto "${CMAKE_CURRENT_SOURCE_DIR}/${proto}")
        cmake_path(GET proto PARENT_PATH proto_dir)
        cmake_path(GET proto STEM proto_stem)
        set(proto_cpp_file "${proto_stem}.pb.cc")
        set(proto_hdr_file "${proto_stem}.pb.h")
        set(proto_dep_file "${proto_stem}.d")

        set(proto_cpp_file_path "${CMAKE_CURRENT_BINARY_DIR}/${proto_cpp_file}")
        set(proto_hdr_file_path "${CMAKE_CURRENT_BINARY_DIR}/${proto_hdr_file}")
        set(proto_dep_file_path "${CMAKE_CURRENT_BINARY_DIR}/${proto_dep_file}")

        add_custom_command(
                OUTPUT ${proto_cpp_file} ${proto_hdr_file}
                COMMAND protobuf::protoc
                ARGS
                        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
                        --proto_path=${proto_dir}
                        --dependency_out=${proto_dep_file}
                        ${proto}
                DEPENDS
                        ${proto} protobuf::protoc
                DEPFILE
                        ${proto_dep_file}
                VERBATIM
                COMMENT "Running CXX protocol buffer compiler on ${proto_orig}"
                )

        target_sources(commonroad_protobuf PRIVATE ${proto_cpp_file_path} ${proto_hdr_file_path})

        set_property(SOURCE ${proto_cpp_file} PROPERTY SKIP_UNITY_BUILD_INCLUSION ON)

        if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
                target_sources(commonroad_protobuf
                INTERFACE
                FILE_SET commonroad_protobuf_headers
                TYPE HEADERS
                FILES ${proto_hdr_file_path}
                )
        endif()
endforeach()


target_include_directories(commonroad_protobuf
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        )

target_link_libraries(commonroad_protobuf
        PUBLIC
        $<BUILD_INTERFACE:protobuf::libprotobuf>
        )

set_property(TARGET commonroad_protobuf PROPERTY POSITION_INDEPENDENT_CODE ON)