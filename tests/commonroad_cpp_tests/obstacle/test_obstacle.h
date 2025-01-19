#pragma once

#include <commonroad_cpp/geometry/rectangle.h>

#include "commonroad_cpp/obstacle/obstacle.h"
#include "test_signal_state.h"
#include "test_state.h"

class ObstacleTestInitialization : public StateTestInitialization, public SignalStateTestInitialization {
  protected:
    size_t idObstacleOne;
    ObstacleRole roleObstacleOne;
    ObstacleType obstacleTypeObstacleOne;
    double vMaxObstacleOne;
    double aMaxObstacleOne;
    double aMaxLongObstacleOne;
    double aMinLongObstacleOne;
    double reactionTimeObstacleOne;
    double lengthObstacleOne;
    double widthObstacleOne;
    std::map<size_t, std::shared_ptr<Lane>> occupiedLaneObstacleOne{};
    state_map_t trajectoryPredictionObstacleOne{};
    state_map_t historyObstacleOne{};
    Rectangle geoShapeObstacleOne;
    std::map<size_t, std::vector<std::shared_ptr<Lanelet>>> occupiedLaneletsObstacleOne{};
    std::shared_ptr<Obstacle> obstacleOne;

    size_t idObstacleTwo;
    ObstacleRole roleObstacleTwo;
    ObstacleType obstacleTypeObstacleTwo;
    double vMaxObstacleTwo;
    double aMaxObstacleTwo;
    double aMaxLongObstacleTwo;
    double aMinLongObstacleTwo;
    double reactionTimeObstacleTwo;
    double lengthObstacleTwo;
    double widthObstacleTwo;
    state_map_t trajectoryPredictionObstacleTwo{};
    std::shared_ptr<Obstacle> obstacleTwo;

    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    std::shared_ptr<Obstacle> obstacleSix;
    std::shared_ptr<Obstacle> obstacleSeven;

    std::vector<std::shared_ptr<Obstacle>> obstacleList{};

    void setUpObstacles();

    static void compareStates(const std::shared_ptr<State> &stateOne, const std::shared_ptr<State> &stateTwo);
};

class ObstacleTest : public ObstacleTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
