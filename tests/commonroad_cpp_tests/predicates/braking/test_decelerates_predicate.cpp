#include "test_decelerates_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void TestDeceleratePredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 2, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 30, 2, 10, 1, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 40, 2, 11, 1, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 51, 2, 12, 1, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 63, 2, 13, 1, 0);

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 0, 2, 10, -1, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 2, 9, -2, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 19, 2, 7, -1, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 26, 2, 6, 1, 0);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(3, 32, 2, 7, 0, 0);
    std::shared_ptr<State> stateFiveObstacleTwo = std::make_shared<State>(3, 39, 2, 7, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};
    world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                      std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, 0.1));
}

TEST_F(TestDeceleratePredicate, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleTwo));
}

TEST_F(TestDeceleratePredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestDeceleratePredicate, RobustEvaluation) {
    EXPECT_EQ(pred.robustEvaluation(0, world, obstacleOne), 0);
    EXPECT_EQ(pred.robustEvaluation(1, world, obstacleOne), 0);
    EXPECT_EQ(pred.robustEvaluation(2, world, obstacleOne), 1);
    EXPECT_EQ(pred.robustEvaluation(3, world, obstacleOne), 1);
    EXPECT_EQ(pred.robustEvaluation(4, world, obstacleOne), 1);
    EXPECT_EQ(pred.robustEvaluation(5, world, obstacleOne), 1);

    EXPECT_EQ(pred.robustEvaluation(0, world, obstacleTwo), -1);
    EXPECT_EQ(pred.robustEvaluation(1, world, obstacleTwo), -2);
    EXPECT_EQ(pred.robustEvaluation(2, world, obstacleTwo), -1);
    EXPECT_EQ(pred.robustEvaluation(3, world, obstacleTwo), 1);
    EXPECT_EQ(pred.robustEvaluation(4, world, obstacleTwo), 0);
    EXPECT_EQ(pred.robustEvaluation(5, world, obstacleTwo), 0);
}
