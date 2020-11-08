//
// Created by Sebastian Maierhofer on 30.10.20.
//
#pragma once

#ifndef ENV_MODEL_XML_READER_H
#define ENV_MODEL_XML_READER_H

#include "../../road_network/lanelet/lanelet.h"
#include "../../auxiliaryDefs/structs.h"
#include "../../obstacle/obstacle.h"
#include "../../road_network/intersection/intersection.h"
#include "pugi_xml/pugixml.hpp"
#include <cstddef>
#include <vector>

namespace XMLReader {
    // creates all obstacle objects from the XML input
    std::vector<std::shared_ptr<Obstacle>> createObstacleFromXML(const std::string &xmlFile);

    // creates all lanelet objects from the XML input
    std::vector<std::shared_ptr<Lanelet>> createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> trafficSigns, std::vector<std::shared_ptr<TrafficLight>> trafficLights);

    // creates all traffic sign objects from the XML input
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSignFromXML(const std::string &xmlFile);

    // creates all traffic light objects from the XML input
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLightFromXML(const std::string &xmlFile);

    // creates all intersection objects from the XML input
    std::vector<std::shared_ptr<Intersection>> createIntersectionFromXML(const std::string &xmlFile, const std::vector<std::shared_ptr<Lanelet>>& lanelets);

    State extractInitialState(const pugi::xml_node &child);

    State extractState(const pugi::xml_node &states);
}

#endif //ENV_MODEL_XML_READER_H
