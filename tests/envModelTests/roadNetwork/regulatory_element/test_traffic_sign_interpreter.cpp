//
// Created by Bowen Wu.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_traffic_sign_interpreter.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_interpreter.h"
void TrafficSignInterpreterTest::SetUp() {
    // TODO replace with custom road network without loading xml files
    // Read and parse CommonRoad scenario file
    std::string xmlFilePath;
    std::string pathToTestFile;

    xmlFilePath = std::string{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    trafficSigns2020a = XMLReader::createTrafficSignFromXML(xmlFilePath);
    trafficLights2020a = XMLReader::createTrafficLightFromXML(xmlFilePath);
    lanelets2020a = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns2020a, trafficLights2020a);
    obstacles2020a = XMLReader::createObstacleFromXML(xmlFilePath);
    intersections2020a = XMLReader::createIntersectionFromXML(xmlFilePath, lanelets2020a);
    roadNetwork2020a = std::make_shared<RoadNetwork>(lanelets2020a, SupportedTrafficSignCountry::USA,
                                                     intersections2020a, trafficSigns2020a, trafficLights2020a);
    xmlFilePath = std::string{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    trafficSigns2018b = XMLReader::createTrafficSignFromXML(xmlFilePath);
    trafficLights2018b = XMLReader::createTrafficLightFromXML(xmlFilePath);
    lanelets2018b = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns2018b, trafficLights2018b);
    obstacles2018b = XMLReader::createObstacleFromXML(xmlFilePath);
    intersections2018b = XMLReader::createIntersectionFromXML(xmlFilePath, lanelets2018b);
    roadNetwork2018b = std::make_shared<RoadNetwork>(lanelets2018b, SupportedTrafficSignCountry::GERMANY,
                                                     intersections2018b, trafficSigns2018b, trafficLights2018b);
}

TEST_F(TrafficSignInterpreterTest, SpeedLimit) {
    TrafficSignInterpreter interp(SupportedTrafficSignCountry::USA);
    std::shared_ptr<Lanelet> lanelet3419 =
        *std::find_if(lanelets2020a.begin(), lanelets2020a.end(), [](auto &lptr) { return lptr->getId() == 3419; });
    std::shared_ptr<Lanelet> lanelet3489 =
        *std::find_if(lanelets2020a.begin(), lanelets2020a.end(), [](auto &lptr) { return lptr->getId() == 3489; });
    std::set<size_t> ids{3432, 3440, 3492};

    EXPECT_NEAR(interp.speedLimit(*lanelet3419), 13.4112, 0.001);
    EXPECT_NEAR(interp.speedLimit(*lanelet3489), 11.176, 0.001);
    EXPECT_NEAR(interp.speedLimit(ids, roadNetwork2020a), 11.176, 0.001);
}

// TEST_F(TrafficSignInterpreterTest, RequiredSpeed) {
//    // TODO: env model does not seem to have a scenario with min speed
//}