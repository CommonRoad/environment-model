set(LIBRARY_NAME env_model)

set(ENV_MODEL_SRC_FILES
        commonroad_cpp/roadNetwork/road_network.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet.cpp
        commonroad_cpp/roadNetwork/lanelet/lane.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_operations.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/stop_line.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.cpp
        commonroad_cpp/roadNetwork/intersection/incoming.cpp
        commonroad_cpp/roadNetwork/intersection/intersection.cpp
        commonroad_cpp/obstacle/obstacle.cpp
        commonroad_cpp/obstacle/obstacle_operations.cpp
        commonroad_cpp/obstacle/state.cpp
        commonroad_cpp/geometry/circle.cpp
        commonroad_cpp/geometry/geometric_operations.cpp
        commonroad_cpp/geometry/rectangle.cpp
        commonroad_cpp/interfaces/commonroad/pugi_xml/pugixml.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2018b.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2020a.cpp
        commonroad_cpp/interfaces/commonroad/xml_reader.cpp
        commonroad_cpp/world.cpp
        commonroad_cpp/predicates/commonroad_predicate.cpp
        commonroad_cpp/predicates/braking/safe_distance_predicate.cpp
        commonroad_cpp/predicates/braking/unnecessary_braking_predicate.cpp
        commonroad_cpp/predicates/position/in_front_of_predicate.cpp
        commonroad_cpp/predicates/position/in_same_lane_predicate.cpp
        commonroad_cpp/predicates/general/cut_in_predicate.cpp
        commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.cpp
        commonroad_cpp/commonroad_container.cpp
        commonroad_cpp/predicates/predicate_parameters.cpp
        commonroad_cpp/auxiliaryDefs/timer.cpp
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
add_executable(env_model_example_node commonroad_cpp/interfaces/standalone/main.cpp)
target_link_libraries(env_model_example_node PRIVATE ${LIBRARY_NAME})


# add pybind module
if(BUILD_PYBIND)
    pybind11_add_module(cpp_env_model
            commonroad_cpp/interfaces/pybind/python_interface.cpp
            commonroad_cpp/interfaces/pybind/python_interface.h
            commonroad_cpp/interfaces/pybind/translate_python_types.cpp
            commonroad_cpp/interfaces/pybind/translate_python_types.h)
    target_link_libraries(cpp_env_model PRIVATE ${LIBRARY_NAME})
endif(BUILD_PYBIND)

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


