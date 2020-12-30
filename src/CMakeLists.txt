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

# Create environment model library
#set(LIB_CR_ENV_MODEL_SRC ${ENV_MODEL_SRC_FILES})
add_library(env_model STATIC ${ENV_MODEL_SRC_FILES})
target_compile_features(env_model
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET env_model PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
target_include_directories(env_model PUBLIC "." ${LIB_CRCCOSY_INCLUDE})
target_link_libraries (env_model
        ${CRCCOSY_LIBRARY}
        Boost::filesystem
        Boost::program_options
        OpenMP::OpenMP_CXX
        Eigen3::Eigen)

# Create environment model test node
add_executable(env_model_example_node interfaces/standalone/main.cpp)
target_compile_features(env_model_example_node
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET env_model_example_node PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET env_model_example_node PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
target_link_libraries (env_model_example_node env_model)
