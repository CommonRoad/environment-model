#include "test_in_lanelet_driving_dir_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void InLaneletDrivingDirPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroVehicleOne = std::make_shared<State>(0, 0, 1, 10, 0, 0);
    std::shared_ptr<State> stateOneVehicleOne = std::make_shared<State>(1, 10, 1, 10, 0, 0);
    Obstacle::state_map_t trajectoryPredictionVehicleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroVehicleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneVehicleOne),
    };
    vehicleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroVehicleOne, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionVehicleOne, 5, 2));

    std::shared_ptr<State> stateZeroVehicleTwo = std::make_shared<State>(0, 12, 1, 3, 0, 0);
    std::shared_ptr<State> stateOneVehicleTwo = std::make_shared<State>(1, 15, 1, 3, 0, 0);
    Obstacle::state_map_t trajectoryPredictionVehicleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroVehicleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneVehicleTwo),
    };
    vehicleTwo =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateOneVehicleTwo, ObstacleType::bicycle, 10, 10,
                                            3, -10, 0.3, trajectoryPredictionVehicleTwo, 2, 0.5));

    std::shared_ptr<State> stateZeroVehicleThree = std::make_shared<State>(0, 10, 1, 10, 0, M_PI);
    std::shared_ptr<State> stateOneVehicleThree = std::make_shared<State>(1, 0, 1, 10, 0, M_PI);
    Obstacle::state_map_t trajectoryPredictionVehicleThree{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroVehicleThree),
        std::pair<int, std::shared_ptr<State>>(1, stateOneVehicleThree),
    };
    vehicleThree =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroVehicleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionVehicleThree, 5, 2));

    std::shared_ptr<State> stateZeroVehicleFour = std::make_shared<State>(0, 15, 1, 3, 0, M_PI);
    std::shared_ptr<State> stateOneVehicleFour = std::make_shared<State>(1, 12, 1, 3, 0, M_PI);
    Obstacle::state_map_t trajectoryPredictionVehicleFour{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroVehicleFour),
        std::pair<int, std::shared_ptr<State>>(1, stateOneVehicleFour),
    };
    vehicleFour =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateOneVehicleFour, ObstacleType::bicycle, 10,
                                            10, 3, -10, 0.3, trajectoryPredictionVehicleFour, 2, 0.5));

    std::shared_ptr<State> stateZeroVehicleFive = std::make_shared<State>(0, 0, 1, 10, 0, M_PI / 8);
    std::shared_ptr<State> stateOneVehicleFive = std::make_shared<State>(1, 10, 1, 10, 0, M_PI / 8);
    Obstacle::state_map_t trajectoryPredictiVehicleFive{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroVehicleFive),
        std::pair<int, std::shared_ptr<State>>(1, stateOneVehicleFive),
    };
    vehicleFive = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroVehicleFive, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictiVehicleFive, 5, 2));
}

TEST_F(InLaneletDrivingDirPredicateTest, BooleanEvaluationOneWayVehicle) {
    initializeTestData(std::set<ObstacleType>{ObstacleType::car}, {});

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleOne));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleTwo));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, vehicleThree));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, vehicleThree));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, vehicleFour));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, vehicleFour));
}

TEST_F(InLaneletDrivingDirPredicateTest, BooleanEvaluationOneWayCar) {
    initializeTestData(std::set<ObstacleType>{ObstacleType::car}, {});

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleOne));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleOne));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, vehicleThree));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, vehicleThree));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, vehicleFour));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, vehicleFour));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleFive));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleFive));
}

TEST_F(InLaneletDrivingDirPredicateTest, BooleanEvaluationBidirectionalVehicle) {
    initializeTestData({}, std::set<ObstacleType>{ObstacleType::car});

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleOne));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleTwo));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleThree));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleThree));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleFour));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleFour));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleFive));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleFive));
}

TEST_F(InLaneletDrivingDirPredicateTest, BooleanEvaluationBidirectionalBicycle) {
    initializeTestData(std::set<ObstacleType>{ObstacleType::car}, std::set<ObstacleType>{ObstacleType::bicycle});

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleOne));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleTwo));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, vehicleFive));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, vehicleFive));
}

void InLaneletDrivingDirPredicateTest::initializeTestData(const std::set<ObstacleType> &userOneWayLanelet,
                                                          const std::set<ObstacleType> &userBidirectionalLanelet) {
    auto roadNetwork{utils_predicate_test::create_road_network_users(userOneWayLanelet, userBidirectionalLanelet)};
    this->world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {this->vehicleOne},
                                                {this->vehicleTwo, this->vehicleThree, this->vehicleFour}, 0.1));
}

TEST_F(InLaneletDrivingDirPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, vehicleOne), std::runtime_error);
}

TEST_F(InLaneletDrivingDirPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, vehicleOne), std::runtime_error);
}
