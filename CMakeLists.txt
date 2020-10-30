cmake_minimum_required(VERSION 3.13)
project(env_model)

set(CMAKE_CXX_STANDARD 17)

add_executable(env_model interfaces/standalone/main.cpp)
#        road_network/lanelet/lanelet.cpp
#        road_network/lanelet/lanelet.h
#        auxiliaryDefs/structs.h
#        interfaces/commonroad/XMLReader.h
#        interfaces/commonroad/XMLReader.cpp
#        interfaces/commonroad/CommonRoadFactory.h
#        interfaces/commonroad/CommonRoadFactory2018b.cpp
#        interfaces/commonroad/CommonRoadFactory2018b.h
#        interfaces/commonroad/CommonRoadFactory2020a.h
#        interfaces/commonroad/CommonRoadFactory2020a.cpp
#        interfaces/commonroad/pugi_xml/pugixml.cpp
#        interfaces/commonroad/pugi_xml/pugiconfig.hpp
#        interfaces/commonroad/pugi_xml/pugixml.hpp)