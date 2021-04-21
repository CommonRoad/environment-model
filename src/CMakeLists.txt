set(ENV_MODEL_SRC_FILES
        roadNetwork/road_network.cpp
        roadNetwork/lanelet/lanelet.cpp
        roadNetwork/lanelet/lane.cpp
        roadNetwork/lanelet/lanelet_operations.cpp
        roadNetwork/regulatoryElements/stop_line.cpp
        roadNetwork/regulatoryElements/traffic_light.cpp
        roadNetwork/regulatoryElements/traffic_sign.cpp
        roadNetwork/regulatoryElements/traffic_sign_element.cpp
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
        world.cpp
        predicates/predicate.cpp
        predicates/safe_distance_predicate.cpp
        predicates/in_front_of_predicate.cpp
        predicates/in_same_lane_predicate.cpp
        )

# Create environment model library
add_library(env_model SHARED ${ENV_MODEL_SRC_FILES})

target_include_directories(env_model PUBLIC "." ${LIB_CRCCOSY_INCLUDE})
target_link_libraries(env_model PUBLIC
        ${CRCCOSY_LIBRARY}
        Boost::filesystem
        Boost::program_options
        OpenMP::OpenMP_CXX
        Eigen3::Eigen)

# Create environment model test node
add_executable(env_model_example_node interfaces/standalone/main.cpp)

target_link_libraries(env_model_example_node PRIVATE env_model)


# add pybind module
pybind11_add_module(cpp_env_model
        interfaces/pybind/python_interface.cpp
        interfaces/pybind/python_interface.h
        interfaces/pybind/translate_python_types.cpp
        interfaces/pybind/translate_python_types.h)
target_link_libraries(cpp_env_model PRIVATE env_model)
