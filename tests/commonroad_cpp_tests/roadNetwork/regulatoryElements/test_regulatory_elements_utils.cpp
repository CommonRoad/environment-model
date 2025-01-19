#include "test_regulatory_elements_utils.h"
#include "../../predicates/utils_predicate_test.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/roadNetwork/road_network.h"

void RegulatoryElementsUtilsTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 45, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 45, 2, 50, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 95, 2, 55, 0, 0, 0, 95, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 150, 2, 45, 0, 0, 0, 150, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_2()};
    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(RegulatoryElementsUtilsTest, TypeSpeedLimit) {
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::truck), 22.22);
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::pedestrian), std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::car), std::numeric_limits<double>::max());
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitSingle) {
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(111),
                                                    TrafficSignTypes::MAX_SPEED),
              35.0);
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(222),
                                                    TrafficSignTypes::MAX_SPEED),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(333),
                                                    TrafficSignTypes::MAX_SPEED),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(444),
                                                    TrafficSignTypes::MAX_SPEED),
              std::numeric_limits<double>::max());
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitVector) {
    EXPECT_EQ(regulatory_elements_utils::speedLimit(
                  {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MAX_SPEED),
              35.0);
    EXPECT_EQ(regulatory_elements_utils::speedLimit(
                  {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)},
                  TrafficSignTypes::MAX_SPEED),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit(
                  {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MAX_SPEED),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit({world->getRoadNetwork()->findLaneletById(111)},
                                                    TrafficSignTypes::MAX_SPEED),
              35.0);
}

TEST_F(RegulatoryElementsUtilsTest, RequiredVelocitySingle) {
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(111),
                                                          TrafficSignTypes::MIN_SPEED),
              10.0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(222),
                                                          TrafficSignTypes::MIN_SPEED),
              std::numeric_limits<double>::lowest());
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(333),
                                                          TrafficSignTypes::MIN_SPEED),
              std::numeric_limits<double>::lowest());
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(444),
                                                          TrafficSignTypes::MIN_SPEED),
              std::numeric_limits<double>::lowest());
}

TEST_F(RegulatoryElementsUtilsTest, RequiredVelocityVector) {
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(
                  {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MIN_SPEED),
              10.0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(
                  {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)},
                  TrafficSignTypes::MIN_SPEED),
              std::numeric_limits<double>::lowest());
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(
                  {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MIN_SPEED),
              std::numeric_limits<double>::lowest());
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity({world->getRoadNetwork()->findLaneletById(111)},
                                                          TrafficSignTypes::MIN_SPEED),
              10.0);
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitSuggested) {
    EXPECT_EQ(regulatory_elements_utils::speedLimitSuggested(
                  {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MAX_SPEED, 36.11),
              35.0);
    EXPECT_EQ(regulatory_elements_utils::speedLimitSuggested(
                  {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)},
                  TrafficSignTypes::MAX_SPEED, 36.11),
              36.11);
    EXPECT_EQ(regulatory_elements_utils::speedLimitSuggested(
                  {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)},
                  TrafficSignTypes::MAX_SPEED, 36.11),
              36.11);
    EXPECT_EQ(regulatory_elements_utils::speedLimitSuggested({world->getRoadNetwork()->findLaneletById(111)},
                                                             TrafficSignTypes::MAX_SPEED, 36.11),
              35.0);
}

TEST_F(RegulatoryElementsUtilsTest, ExtractTypeFromString) {
    EXPECT_EQ(regulatory_elements_utils::extractTypeFromNationalID("274", SupportedTrafficSignCountry::GERMANY, "DEU"),
              TrafficSignTypes::MAX_SPEED);
}

TEST_F(RegulatoryElementsUtilsTest, TrafficSignReferencesStopSign) {}

TEST_F(RegulatoryElementsUtilsTest, AtRedTrafficLight) {}

TEST_F(RegulatoryElementsUtilsTest, ActiveTrafficLights) {}

// TEST_F(RegulatoryElementsUtilsTest, TrafficSignInFront) {
//    EXPECT_TRUE(regulatory_elements_utils::trafficSignInFront(
//        0, world->getRoadNetwork()->findLaneletById(111)->getTrafficSigns().at(0), obstacleOne,
//        world->getRoadNetwork()));
//    EXPECT_FALSE(regulatory_elements_utils::trafficSignInFront(
//        1, world->getRoadNetwork()->findLaneletById(111)->getTrafficSigns().at(0), obstacleOne,
//        world->getRoadNetwork()));
//}

TEST_F(RegulatoryElementsUtilsTest, MatchDirections) {
    EXPECT_EQ(regulatory_elements_utils::matchDirections("right"), Direction::right);
    EXPECT_EQ(regulatory_elements_utils::matchDirections("straight"), Direction::straight);
    EXPECT_EQ(regulatory_elements_utils::matchDirections("left"), Direction::left);
    EXPECT_EQ(regulatory_elements_utils::matchDirections("leftStraight"), Direction::leftStraight);
    EXPECT_EQ(regulatory_elements_utils::matchDirections("straightRight"), Direction::straightRight);
    EXPECT_EQ(regulatory_elements_utils::matchDirections("leftRight"), Direction::leftRight);
    EXPECT_THROW(regulatory_elements_utils::matchDirections("test"), std::logic_error);
}
