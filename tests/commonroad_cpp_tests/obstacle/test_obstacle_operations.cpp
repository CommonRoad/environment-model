//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_obstacle_operations.h"
#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/geometry/geometric_operations.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include "commonroad_cpp/world.h"

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void ObstacleOperationsTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
    for (size_t timeStep{0}; timeStep < 11; ++timeStep)
        obstacleThree->setOccupiedLanes(roadNetwork, timeStep);
    obstacleFour->setOccupiedLanes(roadNetwork, 7);
}

TEST_F(ObstacleOperationsTest, GetObstacleById) {
    EXPECT_EQ(obstacle_operations::getObstacleById(obstacleList, 1)->getId(), 1);
    EXPECT_EQ(obstacle_operations::getObstacleById(obstacleList, 2)->getId(), 2);
}

TEST_F(ObstacleOperationsTest, MatchObstacleTypeToString) {
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("car"), ObstacleType::car);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("truck"), ObstacleType::truck);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("pedestrian"), ObstacleType::pedestrian);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("bus"), ObstacleType::bus);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("unknown"), ObstacleType::unknown);
}

TEST_F(ObstacleOperationsTest, ObstacleDirectlyLeft) {
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(0, obstacleList, obstacleThree, roadNetwork),
              nullptr); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(1, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(2, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(3, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(4, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(5, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(6, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(7, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // two left vehicles
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(8, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(9, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(10, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesLeft) {
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(0, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // static obstacle adjacent right
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(1, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle exactly left of
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(1, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(2, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle partially left of in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(2, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(3, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle partially left of in behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(3, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(4, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle left of in front and behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(4, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(5, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle in same lane in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(6, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).size(),
              2); // other vehicle exactly right of
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 2);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 5);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(8, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(9, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(10, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesAdjacent) {
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(0, obstacleList, obstacleThree, roadNetwork).size(),
              1); // static obstacle adjacent
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(0, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 1);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(1, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(1, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(2, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(2, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(3, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(3, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(4, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(4, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(5, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(6, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).size(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 2);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(2)->getId(), 5);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(8, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(8, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(9, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(9, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(10, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(10, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
}

TEST_F(ObstacleOperationsTest, ObstacleDirectlyRight) {
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(0, obstacleList, obstacleThree, roadNetwork),
              nullptr); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(1, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(2, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(3, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(4, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(5, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(6, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(7, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle exactly right of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(8, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(9, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(10, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesRight) {
    EXPECT_EQ(obstacle_operations::obstaclesRight(0, obstacleList, obstacleThree, roadNetwork).size(),
              0); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstaclesRight(1, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstaclesRight(2, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(3, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(4, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(5, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(6, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).size(),
              2); // other vehicle exactly right of
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 6);
    EXPECT_EQ(obstacle_operations::obstaclesRight(8, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(8, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(9, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(9, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(10, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(10, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
}

TEST_F(ObstacleOperationsTest, LaneletsRightOfObstacle) {
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(0, roadNetwork, obstacleThree).size(), 1);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(1, roadNetwork, obstacleThree).size(), 1);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
}

TEST_F(ObstacleOperationsTest, LaneletsLeftOfObstacle) {
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(0, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(1, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(7, roadNetwork, obstacleFour).size(), 1);
}

TEST_F(ObstacleOperationsTest, DrivingDistanceToCoordinatePointStraights) {
    std::string pathToTestFile =
        TestUtils::getTestScenarioDirectory() +
        "/DEU_TwoLanesWithDifferentOrientation-1/DEU_TwoLanesWithDifferentOrientation-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize2] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 0, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 0, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 0, 10, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 0, 10, 0, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 40, 0, 10, 0, 0);

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10, 10, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 20, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 10, 30, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 10, 40, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 10, 50, 10, 0, M_PI / 2);

    Obstacle::state_map_t trajectoryPredictionOneVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionTwoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo)};

    std::shared_ptr<Obstacle> obstacleOne =
        std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionOneVehicle, 5, 2));

    std::shared_ptr<Obstacle> obstacleTwo =
        std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::bus, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionTwoVehicle, 10, 2));

    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne, obstacleTwo}, {}, 0.1));

    // obstacle one drive horizontally
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(30, 0, roadNetwork, obstacleOne, 0),
              30 - (5.0 / 2));
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(30, 0, roadNetwork, obstacleOne, 1),
              20 - (5.0 / 2));
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(30, 0, roadNetwork, obstacleOne, 2),
              10 - (5.0 / 2));

    // obstacle two drives vertically
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(10, 50, roadNetwork, obstacleTwo, 0),
              40 - (10.0 / 2));
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(10, 50, roadNetwork, obstacleTwo, 1),
              30 - (10.0 / 2));
    ASSERT_EQ(obstacle_operations::drivingDistanceToCoordinatePoint(10, 50, roadNetwork, obstacleTwo, 2),
              20 - (10.0 / 2));
}

TEST_F(ObstacleOperationsTest, DrivingDistanceToCoordinatePoint180Corner) {
    std::string pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/DEU_CornerWith180Degree-1/DEU_CornerWith180Degree-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize2] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    const double cornerRadius = 10.0;
    const double distanceToCover = M_PI * cornerRadius;

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 0, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 7, 3, 10, 0, M_PI / 4);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 10, 10, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 7, 17, 10, 0, M_PI * 3 / 4);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 0, 20, 10, 0, M_PI);

    Obstacle::state_map_t trajectoryPredictionOneVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne)};
    const double obstacle_length = 5.0;

    std::shared_ptr<Obstacle> obstacleOne =
        std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionOneVehicle, 5, 2));

    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne, obstacleTwo}, {}, 0.1));

    // Larger tolerances for the edges of the lanelet, because then the result depends on how we extrapolate its center
    // line
    ASSERT_NEAR(obstacle_operations::drivingDistanceToCoordinatePoint(0, 20, roadNetwork, obstacleOne, 0),
                distanceToCover * 1 - (obstacle_length / 2), 0.26);
    ASSERT_NEAR(obstacle_operations::drivingDistanceToCoordinatePoint(0, 20, roadNetwork, obstacleOne, 1),
                distanceToCover * 0.75 - (obstacle_length / 2), 0.15);
    ASSERT_NEAR(obstacle_operations::drivingDistanceToCoordinatePoint(0, 20, roadNetwork, obstacleOne, 2),
                distanceToCover * 0.5 - (obstacle_length / 2), 0.15);
    ASSERT_NEAR(obstacle_operations::drivingDistanceToCoordinatePoint(0, 20, roadNetwork, obstacleOne, 3),
                distanceToCover * 0.25 - (obstacle_length / 2), 0.15);
    ASSERT_NEAR(obstacle_operations::drivingDistanceToCoordinatePoint(0, 20, roadNetwork, obstacleOne, 4),
                distanceToCover * 0 - (obstacle_length / 2), 0.26);
    // not in projection domain
    ASSERT_THROW(obstacle_operations::drivingDistanceToCoordinatePoint(-20, 20, roadNetwork, obstacleOne, 4),
                 std::invalid_argument);
}
