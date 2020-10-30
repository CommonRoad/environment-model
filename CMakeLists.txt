cmake_minimum_required(VERSION 3.13)
project(env_model)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

add_executable(env_model env_model/interfaces/standalone/main.cpp
        env_model/road_network/lanelet/lanelet.cpp
        env_model/road_network/lanelet/lanelet.h
        env_model/obstacle/obstacle.cpp
        env_model/obstacle/obstacle.h
        env_model/auxiliaryDefs/structs.h
        env_model/interfaces/commonroad/XMLReader.h
        env_model/interfaces/commonroad/XMLReader.cpp
        env_model/interfaces/commonroad/CommonRoadFactory.h
        env_model/interfaces/commonroad/CommonRoadFactory2018b.cpp
        env_model/interfaces/commonroad/CommonRoadFactory2018b.h
        env_model/interfaces/commonroad/CommonRoadFactory2020a.h
        env_model/interfaces/commonroad/CommonRoadFactory2020a.cpp
        env_model/interfaces/commonroad/pugi_xml/pugixml.cpp
        env_model/interfaces/commonroad/pugi_xml/pugiconfig.hpp
        env_model/interfaces/commonroad/pugi_xml/pugixml.hpp
        env_model/geometry/circle.cpp
        env_model/geometry/circle.h
        env_model/geometry/rectangle.cpp
        env_model/geometry/rectangle.h
        env_model/geometry/geometricOperations.cpp
        env_model/geometry/geometricOperations.h)

target_link_libraries (env_model Boost::filesystem Boost::program_options)