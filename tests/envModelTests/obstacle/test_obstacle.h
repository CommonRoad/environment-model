//
// Created by sebastian on 16.12.20.
//

#ifndef ENV_MODEL_TEST_OBSTACLE_H
#define ENV_MODEL_TEST_OBSTACLE_H

#include <gtest/gtest.h>
#include "obstacle/obstacle.h"

class ObstacleTest : public testing::Test {
protected:
    int idOne;

    std::shared_ptr<Obstacle> obstacleOne;

    void setUpObstacles();

private:
    void SetUp() override;
};


#endif //ENV_MODEL_TEST_OBSTACLE_H
