//
// Created by sebastian on 04.11.20.
//
#pragma once

#ifndef ENV_MODEL_COMMONROAD_FACTORY_H
#define ENV_MODEL_COMMONROAD_FACTORY_H



#include "pugi_xml/pugixml.hpp"
#include "../../obstacle/obstacle.h"
#include "../../road_network/lanelet/lanelet.h"
#include <stdexcept>
#include <vector>

class CommonRoadFactory {

public:
    explicit CommonRoadFactory(std::unique_ptr<pugi::xml_document> xmlDocument) { doc = std::move(xmlDocument); }

    virtual std::vector<std::shared_ptr<Obstacle>> createObstacles() = 0;

    virtual std::vector<std::shared_ptr<Lanelet>> createLanelets() = 0;

    virtual std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() = 0;

    virtual std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() = 0;

protected:
    std::unique_ptr<pugi::xml_document> doc;
};


#endif //ENV_MODEL_COMMONROAD_FACTORY_H
