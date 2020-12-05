//
// Created by sebastian on 03.12.20.
//

#ifndef ENV_MODEL_TEST_LANELET_H
#define ENV_MODEL_TEST_LANELET_H

#pragma once
#include <gtest/gtest.h>

#include "roadNetwork/lanelet/lanelet.h"


class LaneletTest : public testing::Test{
    protected:
        int idOne;
        std::vector<LaneletType> laneletTypeOne;
        std::vector<ObstacleType> userOneWayOne;
        std::vector<ObstacleType> userBidirectionalOne;
        std::vector<vertice> centerVerticesOne;
        std::vector<vertice> leftBorderOne;
        std::vector<vertice> rightBorderOne;
        std::shared_ptr<Lanelet> laneletOne;

        int idTwo;
        std::vector<LaneletType> laneletTypeTwo;
        std::vector<vertice> centerVerticesTwo;
        std::vector<vertice> leftBorderTwo;
        std::vector<vertice> rightBorderTwo;
        std::shared_ptr<Lanelet> laneletTwo;

        int idThree;
        std::vector<LaneletType> laneletTypeThree;
        std::vector<vertice> centerVerticesThree;
        std::vector<vertice> leftBorderThree;
        std::vector<vertice> rightBorderThree;
        Lanelet laneletThree;

        Lanelet laneletFour;
        Lanelet laneletFive;

        void SetUp() override;
};

#endif //ENV_MODEL_TEST_LANELET_H
