//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_interfaces.h"

TEST_F(InterfacesTest, Read2018bFileSingleThread) {
    std::string xmlFilePath{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 2); // virtual speed limit signs for lanes who have a speed limit
    EXPECT_EQ(trafficLights.size(), 0);
    EXPECT_EQ(intersections.size(), 0);
    EXPECT_EQ(obstacles.size(), 5);
    EXPECT_EQ(lanelets.size(), 2);

    auto lanelet34782 =
        *std::find_if(lanelets.begin(), lanelets.end(), [](auto &lptr) { return lptr->getId() == 34782; });
    auto virtualSpeedLimitElem = lanelet34782->getTrafficSigns()[0]->getTrafficSignElements()[0];
    EXPECT_EQ(virtualSpeedLimitElem->getId(), TrafficSignIDGermany.at(TrafficSignTypes::MAX_SPEED));
    EXPECT_EQ(virtualSpeedLimitElem->getAdditionalValues()[0], "14");
}

TEST_F(InterfacesTest, Read2020aFileSingleThread) {
    std::string xmlFilePath{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 95);
    EXPECT_EQ(trafficLights.size(), 8);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(obstacles.size(), 24);
    EXPECT_EQ(lanelets.size(), 95);
}

TEST_F(InterfacesTest, SamePredecessors) {
    std::string scenarioName = "ARG_Carcarana-6";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = std::get<1>(scenarioXml)->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = std::get<1>(scenarioPb)->getLaneletNetwork();
    EXPECT_EQ(laneletNetworkXml.size(), laneletNetworkPb.size());

    for (size_t laneletI = 0; laneletI < laneletNetworkXml.size(); laneletI++) {
        const std::vector<std::shared_ptr<Lanelet>> predecessorsXml = laneletNetworkXml[laneletI]->getPredecessors();
        const std::vector<std::shared_ptr<Lanelet>> predecessorsPb = laneletNetworkPb[laneletI]->getPredecessors();
        EXPECT_EQ(predecessorsXml.size(), predecessorsPb.size());
        for (size_t pred_i = 0; pred_i < predecessorsXml.size(); pred_i++)
            EXPECT_EQ(predecessorsXml[pred_i]->getId(), predecessorsPb[pred_i]->getId());
    }
}

TEST_F(InterfacesTest, SameRefTrafficSigns) {
    std::string scenarioName = "USA_Peach-2_1_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = std::get<1>(scenarioXml)->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = std::get<1>(scenarioPb)->getLaneletNetwork();
    EXPECT_EQ(laneletNetworkXml.size(), laneletNetworkPb.size());
    for (size_t laneletI = 0; laneletI < laneletNetworkXml.size(); laneletI++) {
        const std::vector<std::shared_ptr<TrafficSign>> trafficSignsXml =
            laneletNetworkXml[laneletI]->getTrafficSigns();
        const std::vector<std::shared_ptr<TrafficSign>> trafficSignsPb = laneletNetworkPb[laneletI]->getTrafficSigns();
        for (size_t trafficSignI = 0; trafficSignI < trafficSignsXml.size(); trafficSignI++)
            EXPECT_EQ(trafficSignsXml[trafficSignI]->getId(), trafficSignsPb[trafficSignI]->getId());
    }
}

TEST_F(InterfacesTest, SameRoadNetwork) {
    std::string scenarioName = "USA_Lanker-1_1_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);
    const std::shared_ptr<RoadNetwork> roadNetworkXml = std::get<1>(scenarioXml);
    const std::shared_ptr<RoadNetwork> roadNetworkPb = std::get<1>(scenarioPb);

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = roadNetworkXml->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = roadNetworkPb->getLaneletNetwork();
    EXPECT_EQ(laneletNetworkXml.size(), laneletNetworkPb.size());
    for (size_t laneletI = 0; laneletI < laneletNetworkXml.size(); laneletI++)
        EXPECT_EQ(laneletNetworkXml[laneletI]->getId(), laneletNetworkPb[laneletI]->getId());

    const std::vector<std::shared_ptr<TrafficSign>> trafficSignsXml = roadNetworkXml->getTrafficSigns();
    const std::vector<std::shared_ptr<TrafficSign>> trafficSignsPb = roadNetworkPb->getTrafficSigns();
    EXPECT_EQ(trafficSignsXml.size(), trafficSignsPb.size());
    for (size_t trafficSignI = 0; trafficSignI < trafficSignsXml.size(); trafficSignI++)
        EXPECT_EQ(trafficSignsXml[trafficSignI]->getId(), trafficSignsPb[trafficSignI]->getId());

    const std::vector<std::shared_ptr<TrafficLight>> trafficLightsXml = roadNetworkXml->getTrafficLights();
    const std::vector<std::shared_ptr<TrafficLight>> trafficLightsPb = roadNetworkPb->getTrafficLights();
    EXPECT_EQ(trafficLightsXml.size(), trafficLightsPb.size());
    for (size_t trafficLightI = 0; trafficLightI < trafficLightsXml.size(); trafficLightI++)
        EXPECT_EQ(trafficLightsXml[trafficLightI]->getId(), trafficLightsPb[trafficLightI]->getId());

    const std::vector<std::shared_ptr<Intersection>> intersectionsXml = roadNetworkXml->getIntersections();
    const std::vector<std::shared_ptr<Intersection>> intersectionPb = roadNetworkPb->getIntersections();
    for (size_t intersectionI = 0; intersectionI < intersectionsXml.size(); intersectionI++)
        EXPECT_EQ(intersectionsXml[intersectionI]->getId(), intersectionPb[intersectionI]->getId());
}

TEST_F(InterfacesTest, SameStepSize) {
    std::string scenarioName = "DEU_Guetersloh-25_4_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const double stepSizeXml = std::get<2>(scenarioXml);
    const double stepSizePb = std::get<2>(scenarioPb);
    EXPECT_TRUE(geometric_operations::equalValues(stepSizeXml, stepSizePb));
}

TEST_F(InterfacesTest, SameObstacles) {
    std::string scenarioName = "DEU_Muc-2_1_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const std::vector<std::shared_ptr<Obstacle>> obstaclesXml = std::get<0>(scenarioXml);
    const std::vector<std::shared_ptr<Obstacle>> obstaclesPb = std::get<0>(scenarioPb);
    EXPECT_EQ(obstaclesXml.size(), obstaclesPb.size());

    for (size_t obstacleI = 0; obstacleI < obstaclesXml.size(); obstacleI++) {
        const std::shared_ptr<Obstacle> &obstacleXml = obstaclesXml[obstacleI];
        const std::shared_ptr<Obstacle> &obstaclePb = obstaclesPb[obstacleI];

        EXPECT_EQ(obstacleXml->getId(), obstaclePb->getId());
        EXPECT_EQ(obstacleXml->isStatic(), obstaclePb->isStatic());
        EXPECT_EQ(obstacleXml->getObstacleType(), obstaclePb->getObstacleType());
    }
}

TEST_F(InterfacesTest, ReadingAll) {
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/" + "test_reading_all.xml";
    std::string pathToTestPbFile = TestUtils::getTestScenarioDirectory() + "/protobuf/test_reading_all.pb";
    EXPECT_NO_THROW(InputUtils::getDataFromCommonRoad(pathToTestXmlFile));
    EXPECT_NO_THROW(InputUtils::getDataFromCommonRoad(pathToTestPbFile));
}

std::tuple<Scenario, Scenario> InterfacesTest::loadXmlAndPbScenarios(const std::string &name) {
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/" + name + ".xml";
    std::string pathToTestPbFile = TestUtils::getTestScenarioDirectory() + "/protobuf/" + name + ".pb";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);
    const auto &scenarioPb = InputUtils::getDataFromCommonRoad(pathToTestPbFile);

    return std::make_tuple(scenarioXml, scenarioPb);
}

TEST_F(InterfacesTest, SameCrossings) {
    std::string scenarioName = "test_reading_intersection_traffic_sign";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    auto roadNetworkXml = std::get<1>(scenarioXml);
    auto roadNetworkPB = std::get<1>(scenarioPb);

    /* TODO get crossings from incomingGroups
    EXPECT_EQ(roadNetworkXml->getIntersections().at(0)->getCrossings().size(),
              roadNetworkPB->getIntersections().at(0)->getCrossings().size());
    EXPECT_EQ(roadNetworkXml->getIntersections().at(0)->getCrossings().at(0)->getId(),
              roadNetworkPB->getIntersections().at(0)->getCrossings().at(0)->getId());
              */
}
