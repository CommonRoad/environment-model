#include "test_overtaking_allowed_predicate.h"

#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"

void OvertakingAllowedPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 90, 2, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 95, 2, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 100, 2, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 105, 2, 5, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleFour;
    for (int i = 0; i <= 50; ++i) {
        std::shared_ptr<State> newState = std::make_shared<State>(0, i, 1.5, 10, 0, 0);
        trajectoryPredictionObstacleFour.insert(std::pair<int, std::shared_ptr<State>>(i, newState));
    }

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_with_2_successors(
        {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban})};
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne}, {}, 0.1));

    roadNetwork->findLaneletById(100)->setLineMarkingLeft(LineMarking::dashed);
    roadNetwork->findLaneletById(101)->setLineMarkingLeft(LineMarking::dashed);
    roadNetwork->findLaneletById(200)->setLineMarkingLeft(LineMarking::dashed);
    roadNetwork->findLaneletById(201)->setLineMarkingLeft(LineMarking::dashed);
}

TEST_F(OvertakingAllowedPredicateTest, StartAndStop) {
    TrafficSign startSign{1, {std::make_shared<TrafficSignElement>(TrafficSignTypes::NO_OVERTAKING_START)}, {}, true};
    world->getRoadNetwork()->findLaneletById(100)->addTrafficSign(std::make_shared<TrafficSign>(startSign));
    TrafficSign stopSign{1, {std::make_shared<TrafficSignElement>(TrafficSignTypes::NO_OVERTAKING_END)}, {}, true};
    world->getRoadNetwork()->findLaneletById(200)->addTrafficSign(std::make_shared<TrafficSign>(stopSign));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, {}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, {}));
}

TEST_F(OvertakingAllowedPredicateTest, OnlyStop) {
    TrafficSign stopSign{1, {std::make_shared<TrafficSignElement>(TrafficSignTypes::NO_OVERTAKING_END)}, {}, true};
    world->getRoadNetwork()->findLaneletById(200)->addTrafficSign(std::make_shared<TrafficSign>(stopSign));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, {}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, {}));
}

TEST_F(OvertakingAllowedPredicateTest, NoSign) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, {}));
}
TEST_F(OvertakingAllowedPredicateTest, StopThenStart) {
    TrafficSign startSign{1, {std::make_shared<TrafficSignElement>(TrafficSignTypes::NO_OVERTAKING_END)}, {}, true};
    world->getRoadNetwork()->findLaneletById(100)->addTrafficSign(std::make_shared<TrafficSign>(startSign));
    TrafficSign stopSign{1, {std::make_shared<TrafficSignElement>(TrafficSignTypes::NO_OVERTAKING_START)}, {}, true};
    world->getRoadNetwork()->findLaneletById(200)->addTrafficSign(std::make_shared<TrafficSign>(stopSign));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, {}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, {}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}));
}
