#include "test_interfaces.h"
#include "utility_functions.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tuple>

#include "commonroad_cpp/geometry/polygon.h"
#include "commonroad_cpp/geometry/shape_group.h"
#include "commonroad_cpp/obstacle/occupancy.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h"
#include "commonroad_cpp/roadNetwork/road_network.h"

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
    EXPECT_EQ(virtualSpeedLimitElem->getTrafficSignType(), TrafficSignTypes::MAX_SPEED);
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
    std::string scenarioName = "ARG_Carcarana-6_5_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = scenarioXml.roadNetwork->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = scenarioPb.roadNetwork->getLaneletNetwork();
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

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = scenarioXml.roadNetwork->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = scenarioPb.roadNetwork->getLaneletNetwork();
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
    const std::shared_ptr<RoadNetwork> roadNetworkXml = scenarioXml.roadNetwork;
    const std::shared_ptr<RoadNetwork> roadNetworkPb = scenarioPb.roadNetwork;

    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkXml = roadNetworkXml->getLaneletNetwork();
    const std::vector<std::shared_ptr<Lanelet>> laneletNetworkPb = roadNetworkPb->getLaneletNetwork();
    EXPECT_EQ(laneletNetworkXml.size(), laneletNetworkPb.size());
    for (size_t laneletI = 0; laneletI < laneletNetworkXml.size(); laneletI++)
        EXPECT_EQ(laneletNetworkXml[laneletI]->getId(), laneletNetworkPb[laneletI]->getId());

    const std::vector<std::shared_ptr<TrafficSign>> trafficSignsXml = roadNetworkXml->getTrafficSigns();
    const std::vector<std::shared_ptr<TrafficSign>> trafficSignsPb = roadNetworkPb->getTrafficSigns();
    EXPECT_EQ(trafficSignsXml.size(), trafficSignsPb.size());
    for (size_t trafficSignI = 0; trafficSignI < trafficSignsXml.size(); trafficSignI++)
        EXPECT_EQ(trafficSignsXml[trafficSignI]->getTrafficSignElements().at(0)->getTrafficSignType(),
                  trafficSignsPb[trafficSignI]->getTrafficSignElements().at(0)->getTrafficSignType());

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

    const double stepSizeXml = scenarioXml.timeStepSize;
    const double stepSizePb = scenarioPb.timeStepSize;
    EXPECT_TRUE(geometric_operations::equalValues(stepSizeXml, stepSizePb));
}

TEST_F(InterfacesTest, SameObstacles) {
    std::string scenarioName = "DEU_Muc-2_1_T-1";
    const auto &[scenarioXml, scenarioPb] = InterfacesTest::loadXmlAndPbScenarios(scenarioName);

    const std::vector<std::shared_ptr<Obstacle>> obstaclesXml = scenarioXml.obstacles;
    const std::vector<std::shared_ptr<Obstacle>> obstaclesPb = scenarioPb.obstacles;
    EXPECT_EQ(obstaclesXml.size(), obstaclesPb.size());

    for (size_t obstacleI = 0; obstacleI < obstaclesXml.size(); obstacleI++) {
        const std::shared_ptr<Obstacle> &obstacleXml = obstaclesXml[obstacleI];
        const std::shared_ptr<Obstacle> &obstaclePb = obstaclesPb[obstacleI];

        EXPECT_EQ(obstacleXml->getId(), obstaclePb->getId());
        EXPECT_EQ(obstacleXml->isStatic(), obstaclePb->isStatic());
        EXPECT_EQ(obstacleXml->getObstacleType(), obstaclePb->getObstacleType());
    }
}

TEST_F(InterfacesTest, SetBasedPrediction) {
    std::string scenarioName = "USA_Lanker-1_1_S-2";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    const std::vector<std::shared_ptr<Obstacle>> obstaclesXml = scenarioXml.obstacles;

    for (const auto &obstacleXml : obstaclesXml) {
        if (obstacleXml->getId() != 42)
            EXPECT_GT(obstacleXml->getSetBasedPrediction().size(), 0);
        else
            EXPECT_EQ(obstacleXml->getSetBasedPrediction().size(), 0);

        if (obstacleXml->getId() == 1213) {
            EXPECT_EQ(obstacleXml->getSetBasedPrediction().at(1)->getShape()->getType(), ShapeType::polygon);
            EXPECT_EQ(obstacleXml->getSetBasedPrediction().at(2)->getShape()->getType(), ShapeType::shapeGroup);
            auto shapeGroup =
                std::dynamic_pointer_cast<ShapeGroup>(obstacleXml->getSetBasedPrediction().at(6)->getShape());
            auto poly{std::dynamic_pointer_cast<Polygon>(shapeGroup->getShapes().at(0))};
            EXPECT_NO_THROW(poly->getPolygon());
            EXPECT_NO_THROW(obstacleXml->getSetBasedPrediction().at(6)->getOccupancyPolygonShape());
        }
    }
}

TEST_F(InterfacesTest, ReadingAll) {
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/" + "ZAM_TestReadingAll-1_1_T-1.xml";
    std::string pathToTestPbFile =
        TestUtils::getTestScenarioDirectory() + "/ZAM_TestReadingAll-1/ZAM_TestReadingAll-1_1_T-1.pb";
    EXPECT_NO_THROW(InputUtils::getDataFromCommonRoad(pathToTestXmlFile));
    EXPECT_NO_THROW(InputUtils::getDataFromCommonRoad(pathToTestPbFile));
}

std::tuple<Scenario, Scenario> InterfacesTest::loadXmlAndPbScenarios(const std::string &name) {
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, name, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/" + name + ".xml";
    std::string pathToTestPbFile = TestUtils::getTestScenarioDirectory() + "/" + dirName + "/" + name + ".pb";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);
    const auto &scenarioPb = InputUtils::getDataFromCommonRoad(pathToTestPbFile);

    return std::make_tuple(scenarioXml, scenarioPb);
}

TEST_F(InterfacesTest, ReadingIntersectionWithCrossing) {
    std::string pathToTestPbFile =
        TestUtils::getTestScenarioDirectory() + "/DEU_BicycleBothRight-1/DEU_BicycleBothRight-1_1_T-1.pb";
    std::shared_ptr<RoadNetwork> rn{InputUtils::getDataFromCommonRoad(pathToTestPbFile).roadNetwork};
    EXPECT_EQ(rn->getIntersections().at(0)->getCrossingGroups().size(), 4);
    EXPECT_EQ(rn->getIntersections().at(0)->getCrossingGroups().at(0)->getCrossingGroupLanelets().size(), 1);
    EXPECT_EQ(rn->getIntersections().at(0)->getCrossingGroups().at(1)->getIncomingGroupID(), 501);
    EXPECT_EQ(rn->getIntersections().at(0)->getCrossingGroups().at(2)->getOutgoingGroupID(), 1076);

    EXPECT_EQ(rn->getIntersections().at(0)->getIncomingGroups().size(), 4);
    EXPECT_EQ(rn->getIntersections().at(0)->getIncomingGroups().at(0)->getIncomingLanelets().size(), 1);
    EXPECT_EQ(rn->getIntersections().at(0)->getIncomingGroups().at(0)->getOutgoingGroupID(), 1074);

    EXPECT_EQ(rn->getIntersections().at(0)->getOutgoingGroups().size(), 4);
    EXPECT_EQ(rn->getIntersections().at(0)->getOutgoingGroups().at(0)->getOutgoingLanelets().size(), 3);
    EXPECT_EQ(rn->getIntersections().at(0)->getOutgoingGroups().at(0)->getIncomingGroupID(), 500);
}
