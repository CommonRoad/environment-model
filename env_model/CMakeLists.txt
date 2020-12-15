set(ENV_MODEL_SRC_FILES
        roadNetwork/road_network.cpp
        roadNetwork/lanelet/lanelet.cpp
        roadNetwork/lanelet/lane.cpp
        roadNetwork/lanelet/lanelet_operations.cpp
        roadNetwork/regulatory_elements/stop_line.cpp
        roadNetwork/regulatory_elements/traffic_light.cpp
        roadNetwork/regulatory_elements/traffic_sign.cpp
        roadNetwork/regulatory_elements/traffic_sign_element.cpp
        roadNetwork/intersection/incoming.cpp
        roadNetwork/intersection/intersection.cpp
        obstacle/obstacle.cpp
        obstacle/obstacle_operations.cpp
        obstacle/state.cpp
        geometry/circle.cpp
        geometry/geometric_operations.cpp
        geometry/rectangle.cpp
        interfaces/commonroad/pugi_xml/pugixml.cpp
        interfaces/commonroad/commonroad_factory_2018b.cpp
        interfaces/commonroad/commonroad_factory_2020a.cpp
        interfaces/commonroad/xml_reader.cpp
        interfaces/standalone/command_line_input.cpp
        predicates/position_predicates.cpp
        predicates/braking_predicates.cpp
        )
if(NOT EXISTS "${CRCCOSY_LIBRARY_DIR}")
    message(SEND_ERROR "Can't find the CommonRoad-Curvilinear-Coordinate System library in ${CRCCOSY_LIBRARY_DIR}")
endif()
set(CRCCOSY_LIBRARY ${CRCCOSY_LIBRARY_DIR}/libcrccosy.a)
set(LIB_CRCCOSY_INCLUDE ${CRCCOSY_LIBRARY_DIR}/cpp)

set(LIB_CR_ENV_MODEL_SRC ${ENV_MODEL_SRC_FILES})
add_library(env_model ${LIB_CR_ENV_MODEL_SRC})

target_compile_features(env_model
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET env_model PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(env_model PUBLIC "." ${LIB_CRCCOSY_INCLUDE})
target_link_libraries (env_model ${CRCCOSY_LIBRARY}  Boost::filesystem Boost::program_options)


add_executable(test_node interfaces/standalone/main.cpp)
target_compile_features(test_node
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET test_node PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(test_node PUBLIC)
target_link_libraries(test_node PUBLIC env_model)

#option(BUILD_DOC "Build documentation" ON)
#
## check if Doxygen is installed
#find_package(Doxygen)
#if (DOXYGEN_FOUND)
#    # set input and output files
#    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)
#    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#
#    # request to configure the file
#    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#    message("Doxygen build started")
#
#    # note the option ALL which allows to build the docs together with the application
#    add_custom_target( doc_doxygen ALL
#            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#            COMMENT "Generating API documentation with Doxygen"
#            VERBATIM )
#else (DOXYGEN_FOUND)
#    message("Doxygen need to be installed to generate the doxygen documentation")
#endif (DOXYGEN_FOUND)