set(LIBRARY_NAME env_model)

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
        predicates/braking/safe_distance_predicate.cpp
        predicates/position/in_front_of_predicate.cpp
        predicates/position/in_same_lane_predicate.cpp
        predicates/predicate_evaluation.cpp
        )

# Create environment model library
add_library(${LIBRARY_NAME} SHARED ${ENV_MODEL_SRC_FILES})
target_compile_features(${LIBRARY_NAME}
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET ${LIBRARY_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(${LIBRARY_NAME} PUBLIC "." ${LIB_CRCCOSY_INCLUDE})
target_link_libraries(${LIBRARY_NAME} PUBLIC
        ${CRCCOSY_LIBRARY}
        Boost::filesystem
        Boost::program_options
        OpenMP::OpenMP_CXX
        Eigen3::Eigen)

# Create environment model test node
add_executable(env_model_example_node interfaces/standalone/main.cpp)

target_link_libraries(env_model_example_node PRIVATE ${LIBRARY_NAME})


# add pybind module
pybind11_add_module(cpp_env_model
        interfaces/pybind/python_interface.cpp
        interfaces/pybind/python_interface.h
        interfaces/pybind/translate_python_types.cpp
        interfaces/pybind/translate_python_types.h)
target_link_libraries(cpp_env_model PRIVATE ${LIBRARY_NAME})

# this adds many warnings to the build. They usually help to find some bugs
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(env_model_example_node PUBLIC -Wall -Wextra -Wconversion -pedantic
            -Wfatal-errors)
    target_compile_options(${LIBRARY_NAME} PUBLIC -Wall -Wextra -Wconversion
            -pedantic -Wfatal-errors)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(${LIBRARY_NAME} PUBLIC /W3 /WX)
    target_compile_options(HW01Main PUBLIC /W3 /WX)
endif()


