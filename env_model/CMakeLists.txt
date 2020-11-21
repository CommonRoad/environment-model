set(ENV_MODEL_SRC_FILES
        road_network/road_network.cpp
        road_network/lanelet/lanelet.cpp
        road_network/lanelet/lane.cpp
        road_network/lanelet/lanelet_operations.cpp
        road_network/regulatory_elements/stop_line.cpp
        road_network/regulatory_elements/traffic_light.cpp
        road_network/regulatory_elements/traffic_sign.cpp
        road_network/regulatory_elements/traffic_sign_element.cpp
        road_network/intersection/incoming.cpp
        road_network/intersection/intersection.cpp
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


set(LIB_CR_ENV_MODEL_SRC ${ENV_MODEL_SRC_FILES})
#set(LIB_ENV_MODEL_INCLUDE ".")

add_library(env_model ${LIB_CR_ENV_MODEL_SRC})


target_compile_features(env_model
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model PROPERTY POSITION_INDEPENDENT_CODE ON)

set(ENV_MODEL_INCLUDE_FILES
        env_model/road_network/road_network.h
        env_model/road_network/lanelet/lanelet.h
        env_model/road_network/lanelet/lane.h
        env_model/road_network/lanelet/lanelet_operations.h
        env_model/road_network/regulatory_elements/stop_line.h
        env_model/road_network/regulatory_elements/traffic_light.h
        env_model/road_network/regulatory_elements/traffic_sign.h
        env_model/road_network/regulatory_elements/traffic_sign_element.h
        env_model/road_network/intersection/incoming.h
        env_model/road_network/intersection/intersection.h
        env_model/obstacle/obstacle.h
        env_model/obstacle/obstacle_operations.h
        env_model/obstacle/state.h
        env_model/geometry/circle.h
        env_model/geometry/geometric_operations.h
        env_model/geometry/rectangle.h
        env_model/interfaces/commonroad/pugi_xml/pugixml.h
        env_model/interfaces/commonroad/commonroad_factory_2018b.h
        env_model/interfaces/commonroad/commonroad_factory_2020a.h
        env_model/interfaces/commonroad/xml_reader.h
        env_model/interfaces/standalone/command_line_input.h
        env_model/predicates/position_predicates.h
        )

set(LIB_CRCCOSY_INCLUDE ${CRCCOSY_LIBRARY_DIR}/cpp)

target_include_directories(env_model PUBLIC ${ENV_MODEL_INCLUDE_FILES} ${LIB_CRCCOSY_INCLUDE})

#add_executable(env_model ${env_files} env_model/interfaces/standalone/main.cpp)
target_link_libraries (env_model Boost::filesystem Boost::program_options)
target_link_libraries(env_model ${CRCCOSY_LIBRARY})

add_executable(test_node interfaces/standalone/main.cpp)
target_compile_features(test_node
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET test_node PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(test_node PUBLIC ${ENV_MODEL_INCLUDE_FILES})
target_link_libraries(test_node PUBLIC env_model)