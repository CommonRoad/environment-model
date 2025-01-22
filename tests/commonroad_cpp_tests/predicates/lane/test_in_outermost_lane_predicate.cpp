#include "test_in_outermost_lane_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void InOutermostLanePredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 4);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 8, 10, 0, 0, 0, 20, 8);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 12, 10, 0, 0, 0, 30, 12);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(InOutermostLanePredicateTest, BooleanEvaluationLeftMostObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"left"}));
}

TEST_F(InOutermostLanePredicateTest, BooleanEvaluationRightMostObjects) {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 4);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 5, 10, 0, 0, 0, 20, 8);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 6, 10, 0, 0, 0, 30, 10);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 12, 10, 0, 0, 0, 40, 12);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 50, -2, 10, 0, 0, 0, 50, -2);
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 60, 2, 10, 0, 0, 0, 60, 2);

    state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle),
    };

    auto egoVehicle2 =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3,
                                            -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    auto world2 = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));

    EXPECT_TRUE(pred.booleanEvaluation(0, world2, egoVehicle2, {}, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, egoVehicle2, {}, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, egoVehicle2, {}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, egoVehicle2, {}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world2, egoVehicle2, {}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world2, egoVehicle2, {}, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(6, world2, egoVehicle2, {}, {"right"}));
}

TEST_F(InOutermostLanePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(InOutermostLanePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(InOutermostLanePredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{world->findObstacle(100)};
    auto obs2{world->findObstacle(101)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs2, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs2, {}, {"left"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs1, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs2, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs2, {}, {"left"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs1, {}, {"right"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs1, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs2, {}, {"right"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs2, {}, {"left"}, true));
}
