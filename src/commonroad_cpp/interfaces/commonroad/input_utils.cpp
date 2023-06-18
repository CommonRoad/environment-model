//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/interfaces/commonroad/protobuf_reader.h>
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include <iostream>

#include <spdlog/spdlog.h>

namespace {

/**
 * Reads CR scenario from file in xml format.
 *
 * @param xmlFilePath File path
 * @return Scenario
 */
Scenario readFromXMLFile(const std::string &xmlFilePath) {
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

    return std::make_tuple(obstacles, roadNetwork, timeStepSize);
}

/**
 * Reads CR scenario from file in protobuf format.
 *
 * @param pbFilePath File path
 * @return Scenario
 */
Scenario readFromProtobufFile(const std::string &pbFilePath) {
    commonroad_dynamic::CommonRoadDynamic commonRoadDynamicMsg = ProtobufReader::loadDynamicProtobufMessage(pbFilePath);
    commonroad_map::CommonRoadMap commonRoadMapMsg = ProtobufReader::loadMapProtobufMessage(pbFilePath);
    commonroad_scenario::CommonRoadScenario commonRoadScenarioMsg =
        ProtobufReader::loadScenarioProtobufMessage(pbFilePath);

    return ProtobufReader::createCommonRoadFromMessage(commonRoadDynamicMsg, commonRoadMapMsg, commonRoadScenarioMsg);
}

} // namespace

Scenario InputUtils::getDataFromCommonRoad(const std::string &filePath)
// Loads and sets up CR scenario
{
    spdlog::info("Read file: {}", filePath);
    std::vector<std::string> pathSplit;
    auto fileEnding{boost::split(pathSplit, filePath, boost::is_any_of("."))};

    Scenario scenario;
    if (pathSplit.back() == "xml")
        scenario = readFromXMLFile(filePath);
    else if (pathSplit.back() == "pb")
        scenario = readFromProtobufFile(filePath);
    else
        throw std::runtime_error("Invalid file name " + filePath + ": .xml or .pb ending missing!");

    spdlog::info("File successfully read: {}", filePath);
    return scenario;
}