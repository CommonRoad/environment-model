//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_obstacle.h"
#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <cmath>
#include <map>
#include <memory>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

void ObstacleTestInitialization::setUpObstacles() {
    size_t globalID{1234};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    obstacleOne = std::make_shared<Obstacle>(Obstacle());
    idObstacleOne = 1;
    isStaticObstacleOne = false;

    obstacleTypeObstacleOne = ObstacleType::car;
    vMaxObstacleOne = 50.0;
    aMaxObstacleOne = 5.0;
    aMaxLongObstacleOne = 4.9;
    aMinLongObstacleOne = -10.5;
    reactionTimeObstacleOne = 0.5;
    widthObstacleOne = 2.0;
    lengthObstacleOne = 4.0;
    occupiedLaneObstacleOne.insert(std::pair<int, std::shared_ptr<Lane>>(0, laneOne));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(1, stateTwo));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(2, stateThree));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(3, stateFour));
    historyObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(0, stateOne));
    lengthObstacleOne = 4.0;
    widthObstacleOne = 3.0;
    geoShapeObstacleOne.setLength(lengthObstacleOne);
    geoShapeObstacleOne.setWidth(widthObstacleOne);
    occupiedLaneletsObstacleOne.insert(
        std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletOne}));
    occupiedLaneletsObstacleOne.insert(
        std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletFive}));
    obstacleOne->setId(idObstacleOne);
    obstacleOne->setIsStatic(isStaticObstacleOne);
    obstacleOne->setCurrentState(stateTwo);
    obstacleOne->setObstacleType(obstacleTypeObstacleOne);
    obstacleOne->setVmax(vMaxObstacleOne);
    obstacleOne->setAmax(aMaxObstacleOne);
    obstacleOne->setAmaxLong(aMaxLongObstacleOne);
    obstacleOne->setAminLong(aMinLongObstacleOne);
    obstacleOne->setReactionTime(reactionTimeObstacleOne);
    obstacleOne->appendStateToHistory(stateOne);
    obstacleOne->setTrajectoryPrediction(trajectoryPredictionObstacleOne);
    // obstacleOne->setOccupiedLanes({laneOne}, obstacleOne->getFirstTrajectoryTimeStep());
    obstacleOne->computeLanes(roadNetwork, globalIdRef, true);
    obstacleOne->setRectangleShape(lengthObstacleOne, widthObstacleOne);

    idObstacleTwo = 2;
    isStaticObstacleTwo = true;
    obstacleTypeObstacleTwo = ObstacleType::bus;
    vMaxObstacleTwo = 30.0;
    aMaxObstacleTwo = 2.5;
    aMaxLongObstacleTwo = 2.0;
    aMinLongObstacleTwo = -8.0;
    reactionTimeObstacleTwo = 1.5;
    widthObstacleTwo = 2.5;
    lengthObstacleTwo = 10.0;

    obstacleTwo = std::make_shared<Obstacle>(
        Obstacle(idObstacleTwo, isStaticObstacleTwo, stateFive, obstacleTypeObstacleTwo, vMaxObstacleTwo,
                 aMaxObstacleTwo, aMaxLongObstacleTwo, aMinLongObstacleTwo, reactionTimeObstacleTwo,
                 trajectoryPredictionObstacleTwo, lengthObstacleTwo, widthObstacleTwo));
    obstacleTwo->computeLanes(roadNetwork, globalIdRef);

    obstacleList.push_back(obstacleOne);
    obstacleList.push_back(obstacleTwo);
}

void ObstacleTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
}

void ObstacleTestInitialization::compareStates(const std::shared_ptr<State> &stateOne,
                                               const std::shared_ptr<State> &stateTwo) {
    EXPECT_EQ(stateOne->getTimeStep(), stateTwo->getTimeStep());
    EXPECT_EQ(stateOne->getVelocity(), stateTwo->getVelocity());
    EXPECT_EQ(stateOne->getAcceleration(), stateTwo->getAcceleration());
    EXPECT_EQ(stateOne->getLatPosition(), stateTwo->getLatPosition());
    EXPECT_EQ(stateOne->getLonPosition(), stateTwo->getLonPosition());
    EXPECT_EQ(stateOne->getGlobalOrientation(), stateTwo->getGlobalOrientation());
    EXPECT_EQ(stateOne->getXPosition(), stateTwo->getXPosition());
    EXPECT_EQ(stateOne->getYPosition(), stateTwo->getYPosition());
    EXPECT_EQ(stateOne->getValidStates().acceleration, stateTwo->getValidStates().acceleration);
    EXPECT_EQ(stateOne->getValidStates().velocity, stateTwo->getValidStates().velocity);
    EXPECT_EQ(stateOne->getValidStates().xPosition, stateTwo->getValidStates().xPosition);
    EXPECT_EQ(stateOne->getValidStates().yPosition, stateTwo->getValidStates().yPosition);
    EXPECT_EQ(stateOne->getValidStates().globalOrientation, stateTwo->getValidStates().globalOrientation);
    EXPECT_EQ(stateOne->getValidStates().lonPosition, stateTwo->getValidStates().lonPosition);
    EXPECT_EQ(stateOne->getValidStates().latPosition, stateTwo->getValidStates().latPosition);
}

TEST_F(ObstacleTest, InitializationComplete) {
    EXPECT_EQ(obstacleOne->getId(), idObstacleOne);
    EXPECT_EQ(obstacleTwo->getId(), idObstacleTwo);
    EXPECT_EQ(obstacleOne->getIsStatic(), isStaticObstacleOne);
    EXPECT_EQ(obstacleTwo->getIsStatic(), isStaticObstacleTwo);
    compareStates(obstacleOne->getCurrentState(), stateTwo);
    compareStates(obstacleTwo->getCurrentState(), stateFive);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getVmax(), vMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getVmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmax(), aMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmaxLong(), aMaxLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmaxLong(), 0.0);
    EXPECT_EQ(obstacleOne->getAminLong(), aMinLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAminLong(), 0.0);
    EXPECT_EQ(obstacleOne->getReactionTime(), reactionTimeObstacleOne);
    EXPECT_EQ(obstacleTwo->getReactionTime(), reactionTimeObstacleTwo);
    EXPECT_EQ(obstacleOne->getId(), laneOne->getId());
    compareVerticesVector(
        {obstacleOne->getReferenceLane(obstacleOne->getCurrentState()->getTimeStep())->getCenterVertices().front(),
         obstacleOne->getReferenceLane(obstacleOne->getCurrentState()->getTimeStep())->getCenterVertices().back()},
        {laneletThree->getCenterVertices().front(), laneletSix->getCenterVertices().back()});
    for (size_t i = 2; i <= 3; ++i)
        compareStates(trajectoryPredictionObstacleOne.at(i), obstacleOne->getTrajectoryPrediction().at(i));
    EXPECT_EQ(obstacleOne->getTrajectoryLength(), 3);
    EXPECT_EQ(obstacleTwo->getTrajectoryLength(), 0);
}

TEST_F(ObstacleTest, SetIsStatic) {
    obstacleOne->setIsStatic(true);
    EXPECT_EQ(obstacleOne->getVmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmaxLong(), 0.0);
    EXPECT_EQ(obstacleOne->getAminLong(), 0.0);
}

TEST_F(ObstacleTest, AppendStateToPrediction) {
    obstacleOne->appendStateToTrajectoryPrediction(stateFive);
    compareStates(obstacleOne->getStateByTimeStep(stateFive->getTimeStep()), stateFive);
}

TEST_F(ObstacleTest, GetOccupancyPolygonShape) {
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(0).outer().at(0).x(), 0.5);
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(0).outer().at(0).y(), 2);
    EXPECT_DOUBLE_EQ(obstacleOne->getOccupancyPolygonShape(1).outer().at(0).x(), 2);
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(1).outer().at(0).y(), 0.0);
}

TEST_F(ObstacleTest, GetOccupiedLanelets) {
    EXPECT_EQ(obstacleOne->getOccupiedLanelets(roadNetwork, 0).size(), 1);
    EXPECT_EQ(obstacleOne->getOccupiedLanelets(roadNetwork, 1).size(), 1);
    EXPECT_EQ(obstacleTwo->getOccupiedLanelets(roadNetwork, 4).size(), 0);
}

TEST_F(ObstacleTest, FrontS) {
    EXPECT_EQ(obstacleOne->frontS(0), lonPositionStateOne + lengthObstacleOne / 2);
    EXPECT_EQ(obstacleOne->frontS(1), lonPositionStateTwo + widthObstacleOne / 2);
}

TEST_F(ObstacleTest, RearS) {
    EXPECT_EQ(obstacleOne->rearS(0), lonPositionStateOne - lengthObstacleOne / 2);
    EXPECT_EQ(obstacleOne->rearS(1), lonPositionStateTwo - widthObstacleOne / 2);
}

TEST_F(ObstacleTest, GetLonPosition) {
    EXPECT_EQ(obstacleOne->getLonPosition(0), lonPositionStateOne);
    EXPECT_EQ(obstacleOne->getLonPosition(1), lonPositionStateTwo);
    EXPECT_EQ(obstacleOne->getLonPosition(2), lonPositionStateThree);
}

TEST_F(ObstacleTest, GetLatPosition) {
    EXPECT_EQ(obstacleOne->getLatPosition(0), latPositionStateOne);
    EXPECT_EQ(obstacleOne->getLatPosition(1), latPositionStateTwo);
    EXPECT_EQ(obstacleOne->getLatPosition(2), latPositionStateThree);
}

TEST_F(ObstacleTest, GetStateByTimeStep) {
    EXPECT_EQ(obstacleOne->getStateByTimeStep(0)->getTimeStep(), 0);
    EXPECT_EQ(obstacleOne->getStateByTimeStep(1)->getTimeStep(), 1);
    EXPECT_EQ(obstacleOne->getStateByTimeStep(2)->getTimeStep(), 2);
}

TEST_F(ObstacleTest, ConvertPointToCurvilinear) {
    stateOne->setXPosition(2.5);
    stateOne->setYPosition(0.5);
    obstacleOne->convertPointToCurvilinear(0);
    EXPECT_NEAR(stateOne->getLonPosition(), 7.5, 0.0005);
    EXPECT_EQ(stateOne->getLatPosition(), 0.0);
    stateOne->setXPosition(2.5);
    stateOne->setYPosition(-0.75);
    obstacleOne->convertPointToCurvilinear(0);
    EXPECT_NEAR(stateOne->getLonPosition(), 7.5, 0.0005);
    EXPECT_EQ(stateOne->getLatPosition(), -1.25);
}

TEST_F(ObstacleTest, SetReferenceGeneral) {
    size_t timeStep{0};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto obsOneScenarioOne{obstacle_operations::getObstacleById(obstaclesScenarioOne, 1219)};
    obsOneScenarioOne->computeLanes(roadNetworkScenarioOne, globalIdRef);
    std::set<size_t> expRefLaneletsObsOneScenarioOne{3570, 3632, 3652, 3616, 3456, 3462, 3470};
    EXPECT_EQ(expRefLaneletsObsOneScenarioOne, obsOneScenarioOne->getReferenceLane(timeStep)->getContainedLaneletIDs());

    const auto obsTwoScenarioOne{obstacle_operations::getObstacleById(obstaclesScenarioOne, 1214)};
    obsTwoScenarioOne->computeLanes(roadNetworkScenarioOne, globalIdRef);
    std::set<size_t> expRefLaneletsObsTwoScenarioOne{3570, 3632, 3652, 3616, 3456, 3462, 3470};
    EXPECT_EQ(expRefLaneletsObsTwoScenarioOne, obsTwoScenarioOne->getReferenceLane(timeStep)->getContainedLaneletIDs());

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_Guetersloh-25_4_T-1.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo, timeStepSizeTwo] =
        CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    auto obsOneScenarioTwo{obstacle_operations::getObstacleById(obstaclesScenarioTwo, 325)};
    obsOneScenarioTwo->computeLanes(roadNetworkScenarioTwo, globalIdRef);
    EXPECT_EQ(77695, obsOneScenarioTwo->getReferenceLane(timeStep)->getContainedLanelets().front()->getId());

    std::string pathToTestFileThree{TestUtils::getTestScenarioDirectory() + "/USA_Peach-2_1_T-1.xml"};
    const auto &[obstaclesScenarioThree, roadNetworkScenarioThree, timeStepSizeThree] =
        CommandLine::getDataFromCommonRoad(pathToTestFileThree);
    auto obsOneScenarioThree{obstacle_operations::getObstacleById(obstaclesScenarioThree, 500)};
    obsOneScenarioThree->computeLanes(roadNetworkScenarioThree, globalIdRef);
    EXPECT_EQ(53758, obsOneScenarioThree->getReferenceLane(timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(53788, obsOneScenarioThree->getReferenceLane(timeStep)->getContainedLanelets().back()->getId());

    std::string pathToTestFileFour{TestUtils::getTestScenarioDirectory() + "/USA_Peach-4_1_T-1.xml"};
    const auto &[obstaclesScenarioFour, roadNetworkScenarioFour, timeStepSizeFour] =
        CommandLine::getDataFromCommonRoad(pathToTestFileFour);
    auto obsOneScenarioFour{obstacle_operations::getObstacleById(obstaclesScenarioFour, 88)};
    obsOneScenarioFour->computeLanes(roadNetworkScenarioFour, globalIdRef);
    EXPECT_EQ(43349, obsOneScenarioFour->getReferenceLane(timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(43486, obsOneScenarioFour->getReferenceLane(timeStep)->getContainedLanelets().back()->getId());

    timeStep = 200;
    std::string pathToTestFileFive{TestUtils::getTestScenarioDirectory() + "/DEU_Frankfurt-70_12_I-1.xml"};
    const auto &[obstaclesScenarioFive, roadNetworkScenarioFive, timeStepSizeFive] =
        CommandLine::getDataFromCommonRoad(pathToTestFileFive);
    auto obsOneScenarioFive{obstacle_operations::getObstacleById(obstaclesScenarioFive, 30503)};
    obsOneScenarioFive->computeLanes(roadNetworkScenarioFive, globalIdRef);
    EXPECT_EQ(865, obsOneScenarioFive->getReferenceLane(timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(876, obsOneScenarioFive->getReferenceLane(timeStep)->getContainedLanelets().back()->getId());
}
