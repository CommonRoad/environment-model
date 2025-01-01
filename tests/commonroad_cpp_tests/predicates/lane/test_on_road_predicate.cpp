#include "test_on_road_predicate.h"
#include "../utils_predicate_test.h"

void OnRoadPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 50, 2, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 100, 2, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 150, 2, 10, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 190, 2, 10, 0, 0);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 100, 6, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    // To test that the first timestep with a distance to the successor lanelet of 100m evaluates to false
    egoVehicle->setSensorParameters({90.1, 90.1});
}

void OnRoadPredicateTest::initializeTestData(LaneletType laneletTypeRight, LaneletType laneletTypeLeft,
                                             LaneletType laneletTypeSuccessorRight,
                                             LaneletType laneletTypeSuccessorLeft) {
    auto roadNetwork{utils_predicate_test::create_road_network_with_2_successors(
        {laneletTypeRight}, {laneletTypeLeft}, {laneletTypeSuccessorRight}, {laneletTypeSuccessorLeft})};
    this->world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {this->egoVehicle}, {}, 0.1));
}

TEST_F(OnRoadPredicateTest, AllUrban) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::right, LaneletType::right);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, AllCrosswalk) {
    initializeTestData(LaneletType::sidewalk, LaneletType::sidewalk, LaneletType::sidewalk, LaneletType::sidewalk);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, HalfOnCrosswalk) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::sidewalk, LaneletType::sidewalk);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, OnBikelaneSeperatedFromRoadLeft) {
    initializeTestData(LaneletType::sidewalk, LaneletType::bicycleLane, LaneletType::sidewalk,
                       LaneletType::bicycleLane);
    EXPECT_FALSE(pred.booleanEvaluation(5, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, OnBikelaneNotSeperatedFromRoadLeft) {
    initializeTestData(LaneletType::urban, LaneletType::bicycleLane, LaneletType::urban, LaneletType::bicycleLane);
    EXPECT_TRUE(pred.booleanEvaluation(5, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, OnBikelaneSeperatedFromRoadRight) {
    initializeTestData(LaneletType::bicycleLane, LaneletType::sidewalk, LaneletType::bicycleLane,
                       LaneletType::sidewalk);
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(OnRoadPredicateTest, OnBikelaneNotSeperatedFromRoadRight) {
    initializeTestData(LaneletType::bicycleLane, LaneletType::urban, LaneletType::bicycleLane, LaneletType::urban);
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
}
