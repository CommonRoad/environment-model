#include "test_is_vru_predicate.h"
void TestIsVruPredicate::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle)};

    egoVehicle =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, {stateZeroEgoVehicle}, ObstacleType::pedestrian,
                                            50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
}

TEST_F(TestIsVruPredicate, VRU) {
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::bicycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::motorcycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle));
}

TEST_F(TestIsVruPredicate, NotVRU) {
    egoVehicle->setObstacleType(ObstacleType::bus);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::car);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::taxi);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::train);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle));
    egoVehicle->setObstacleType(ObstacleType::truck);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle));
}