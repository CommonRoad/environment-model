//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include <spdlog/spdlog.h>

/**
 * Loads and sets up CR scenario.
 * @param xmlFilePath Path to CommonRoad xml file
 * @return Tuple of obstacles and roadNetwork.
 */
std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>
InputUtils::getDataFromCommonRoad(const std::string &xmlFilePath) {
    spdlog::info("Read file: {}", xmlFilePath);
    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);
    auto country{XMLReader::extractCountryFromXML(xmlFilePath)};

    std::shared_ptr<RoadNetwork> roadNetwork{
        std::make_shared<RoadNetwork>(RoadNetwork(lanelets, country, trafficSigns, trafficLights, intersections))};
    for (const auto &inter : roadNetwork->getIntersections())
        inter->computeMemberLanelets(roadNetwork);

    auto timeStepSize{XMLReader::extractTimeStepSize(xmlFilePath)};
    spdlog::info("File successfully read: {}", xmlFilePath);
    return std::make_tuple(obstacles, roadNetwork, timeStepSize);
}