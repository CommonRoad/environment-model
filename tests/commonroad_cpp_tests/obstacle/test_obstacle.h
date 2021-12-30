//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "test_state.h"

class ObstacleTestInitialization : public StateTestInitialization {
  protected:
    size_t idObstacleOne;
    bool isStaticObstacleOne;
    ObstacleType obstacleTypeObstacleOne;
    double vMaxObstacleOne;
    double aMaxObstacleOne;
    double aMaxLongObstacleOne;
    double aMinLongObstacleOne;
    double reactionTimeObstacleOne;
    double lengthObstacleOne;
    double widthObstacleOne;
    std::map<size_t, std::shared_ptr<Lane>> occupiedLaneObstacleOne{};
    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{};
    std::map<size_t, std::shared_ptr<State>> historyObstacleOne{};
    Rectangle geoShapeObstacleOne;
    std::map<size_t, std::vector<std::shared_ptr<Lanelet>>> occupiedLaneletsObstacleOne{};
    std::shared_ptr<Obstacle> obstacleOne;

    size_t idObstacleTwo;
    bool isStaticObstacleTwo;
    ObstacleType obstacleTypeObstacleTwo;
    double vMaxObstacleTwo;
    double aMaxObstacleTwo;
    double aMaxLongObstacleTwo;
    double aMinLongObstacleTwo;
    double reactionTimeObstacleTwo;
    double lengthObstacleTwo;
    double widthObstacleTwo;
    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{};
    std::shared_ptr<Obstacle> obstacleTwo;

    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    std::shared_ptr<Obstacle> obstacleSix;

    std::vector<std::shared_ptr<Obstacle>> obstacleList{};

    void setUpObstacles();

    static void compareStates(const std::shared_ptr<State> &stateOne, const std::shared_ptr<State> &stateTwo);
};

class ObstacleTest : public ObstacleTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
