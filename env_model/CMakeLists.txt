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