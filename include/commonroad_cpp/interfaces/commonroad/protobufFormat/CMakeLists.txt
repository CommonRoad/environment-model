#set(Protobuf_USE_STATIC_LIBS ON)
#find_package(Protobuf)
# protobuf_generate

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

if(POLICY CMP0116)
        cmake_policy(SET CMP0116 NEW)
endif()

add_library(commonroad_protobuf OBJECT)
foreach(proto ${COMMONROAD_PROTOS})
        set(proto_orig "${proto}")
        set(proto "${CMAKE_CURRENT_SOURCE_DIR}/${proto}")
        cmake_path(GET proto PARENT_PATH proto_dir)
        cmake_path(GET proto STEM proto_stem)
        set(proto_cpp_file "${proto_stem}.pb.cc")
        set(proto_hdr_file "${proto_stem}.pb.h")
        set(proto_dep_file "${proto_stem}.d")

        add_custom_command(
                OUTPUT ${proto_cpp_file} ${proto_hdr_file}
                COMMAND protobuf::protoc
                ARGS
                        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
                        --proto_path=${proto_dir}
                        --dependency_out=${proto_dep_file}
                        ${proto}
                DEPENDS
                        ${proto} # protobuf::protoc
                DEPFILE
                        ${proto_dep_file}
                # USES_TERMINAL
                VERBATIM
                COMMENT "Running CXX protocol buffer compiler on ${proto_orig}"

                )
        #list(APPEND PROTO_CPP ${proto_cpp_file} ${proto_hdr_file})

        target_sources(commonroad_protobuf PRIVATE ${proto_cpp_file})
        target_sources(commonroad_protobuf
            INTERFACE
            FILE_SET commonroad_protobuf_headers
            TYPE HEADERS
            FILES ${proto_hdr_file}
            )
        # target_sources(commonroad_protobuf INTERFACE ${proto_hdr_file})
endforeach()


# protobuf_generate_cpp(
#         COMMONROAD_PB_SRCS
#         COMMONROAD_PB_HDRS
#         ${COMMONROAD_PROTOS})

#add_library(commonroad_protobuf OBJECT
#    ${COMMONROAD_PB_SRCS}
#    ${COMMONROAD_PB_HDRS}
#    ${PROTO_CPP}
#    # ${COMMONROAD_PROTOS}
#    )
#protobuf_generate(TARGET commonroad_protobuf)

target_include_directories(commonroad_protobuf
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        )

target_link_libraries(commonroad_protobuf
        PUBLIC
        $<BUILD_INTERFACE:protobuf::libprotobuf>
        )

set_property(TARGET commonroad_protobuf PROPERTY POSITION_INDEPENDENT_CODE ON)