//
// Created by sebastian on 16.12.20.
//

#ifndef ENV_MODEL_TEST_OBSTACLE_H
#define ENV_MODEL_TEST_OBSTACLE_H

#include "obstacle/obstacle.h"
#include "test_state.h"

class ObstacleTestInitialization : public StateTestInitialization {
protected:
    int idObstacleOne;
    bool isStaticObstacleOne;
    ObstacleType obstacleTypeObstacleOne;
    double vMaxObstacleOne;
    double aMaxObstacleOne;
    double aMaxLongObstacleOne;
    double aMinLongObstacleOne;
    double reactionTimeObstacleOne;
    double lengthObstacleOne;
    double widthObstacleOne;
    std::map<int, std::shared_ptr<Lane>> occupiedLaneObstacleOne{};
    std::map<int, State> trajectoryPredictionObstacleOne{};
    std::map<int, State> historyObstacleOne{};
    Rectangle geoShapeObstacleOne;
    std::map<int, std::vector<std::shared_ptr<Lanelet>>> occupiedLaneletsObstacleOne{};
    std::shared_ptr<Obstacle> obstacleOne;

    int idObstacleTwo;
    bool isStaticObstacleTwo;
    ObstacleType obstacleTypeObstacleTwo;
    double vMaxObstacleTwo;
    double aMaxObstacleTwo;
    double aMaxLongObstacleTwo;
    double aMinLongObstacleTwo;
    double reactionTimeObstacleTwo;
    double lengthObstacleTwo;
    double widthObstacleTwo;
    std::map<int, State> trajectoryPredictionObstacleTwo{};
    std::shared_ptr<Obstacle> obstacleTwo;

    std::vector<std::shared_ptr<Obstacle>> obstacleList{};

    void setUpObstacles();

    static void compareStates(State stateOne, State stateTwo);
};

class ObstacleTest : public ObstacleTestInitialization, public testing::Test{
private:
    void SetUp() override;
};


#endif //ENV_MODEL_TEST_OBSTACLE_H
