//
// Created by sebastian on 30.10.20.
//
#pragma once

#ifndef ENV_MODEL_XMLREADER_H
#define ENV_MODEL_XMLREADER_H

#include "../../auxiliaryDefs/structs.h"
#include "../../road_network/lanelet/lanelet.h"
#include "../../obstacle/obstacle.h"
#include <cstddef>
#include <memory>
#include <vector>

namespace XMLReader {
    // creates all obstacle objects from the XML input
    std::vector<std::shared_ptr<Obstacle>> createObstacleFromXML(const std::string &xmlFile);

    // creates all lanelet objects from the XML input
    std::vector<std::shared_ptr<Lanelet>> createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> sign, std::vector<std::shared_ptr<TrafficLight>> light);

    // creates all traffic sign objects from the XML input
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSignFromXML(const std::string &xmlFile);

    // creates all traffic light objects from the XML input
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLightFromXML(const std::string &xmlFile);
}

#endif //ENV_MODEL_XMLREADER_H
