//
// Created by Sebastian Maierhofer on 30.10.20.
//
#pragma once

#ifndef ENV_MODEL_XML_READER_H
#define ENV_MODEL_XML_READER_H

#include "../../roadNetwork/lanelet/lanelet.h"
#include "../../auxiliaryDefs/structs.h"
#include "../../obstacle/obstacle.h"
#include "../../roadNetwork/intersection/intersection.h"
#include "pugi_xml/pugixml.hpp"


namespace XMLReader {
    // creates all obstacle objects from the XML input
    std::vector<std::shared_ptr<Obstacle>> createObstacleFromXML(const std::string &xmlFile);

    // creates all lanelet objects from the XML input
    std::vector<std::shared_ptr<Lanelet>> createLaneletFromXML(
            const std::string &xmlFile,
            std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
            std::vector<std::shared_ptr<TrafficLight>> trafficLights);

    // creates all traffic sign objects from the XML input
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSignFromXML(const std::string &xmlFile);

    // creates all traffic light objects from the XML input
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLightFromXML(const std::string &xmlFile);

    // creates all intersection objects from the XML input
    std::vector<std::shared_ptr<Intersection>> createIntersectionFromXML(
            const std::string &xmlFile,
            const std::vector<std::shared_ptr<Lanelet>>& lanelets);

    State extractInitialState(const pugi::xml_node &child);

    State extractState(const pugi::xml_node &states);

    void createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                               const pugi::xml_node &roadElements);

    void extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                               const pugi::xml_node &roadElements);

    int initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                           const pugi::xml_node &commonRoad);

   void extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                      int arrayIndex,
                                      const pugi::xml_node &child,
                                      const char* side);


    void extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                              int n,
                              int arrayIndex,
                              const pugi::xml_node &child,
                              const char* type);

    void extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                 int n,
                                 int arrayIndex,
                                 const pugi::xml_node &child,
                                 const char* type);
}

#endif //ENV_MODEL_XML_READER_H
