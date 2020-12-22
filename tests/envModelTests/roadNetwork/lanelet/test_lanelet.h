//
// Created by sebastian on 03.12.20.
//

#ifndef ENV_MODEL_TEST_LANELET_H
#define ENV_MODEL_TEST_LANELET_H
#pragma once
#include <gtest/gtest.h>

#include "roadNetwork/lanelet/lanelet.h"


class LaneletTestInitialization{
protected:
    int idLaneletOne;
    std::vector<LaneletType> laneletTypeLaneletOne;
    std::vector<ObstacleType> userOneWayLaneletOne;
    std::vector<ObstacleType> userBidirectionalLaneletOne;
    std::vector<vertice> centerVerticesLaneletOne;
    std::vector<vertice> leftBorderLaneletOne;
    std::vector<vertice> rightBorderLaneletOne;
    std::shared_ptr<Lanelet> laneletOne;

    int idLaneletTwo;
    std::vector<LaneletType> laneletTypeLaneletTwo;
    std::vector<vertice> centerVerticesLaneletTwo;
    std::vector<vertice> leftBorderLaneletTwo;
    std::vector<vertice> rightBorderLaneletTwo;
    std::shared_ptr<Lanelet> laneletTwo;
    std::vector<ObstacleType> userOneWayLaneletTwo;
    std::vector<ObstacleType> userBidirectionalLaneletTwo;

    int idLaneletThree;
    std::vector<LaneletType> laneletTypeLaneletThree;
    std::vector<vertice> centerVerticesLaneletThree;
    std::vector<vertice> leftBorderLaneletThree;
    std::vector<vertice> rightBorderLaneletThree;
    std::shared_ptr<Lanelet> laneletThree;

    std::shared_ptr<Lanelet> laneletFour;
    std::shared_ptr<Lanelet> laneletFive;
    std::shared_ptr<Lanelet> laneletSix;
    std::shared_ptr<Lanelet> laneletSeven;

    polygon_type polygonOne;
    polygon_type polygonTwo;
    polygon_type polygonThree;

    static void compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo);
    template <typename T>
    void evaluateTypes(std::vector<T> typeVectorOne, std::vector<T> typeVectorTwo);

    void setUpLanelets();
};

class LaneletTest : public LaneletTestInitialization, public testing::Test{
private:
    void SetUp() override;
};

template <typename T>
void LaneletTestInitialization::evaluateTypes(std::vector<T> typeVectorOne, std::vector<T> typeVectorTwo) {
    for(auto ty : typeVectorOne){
        bool typeValid{ false };
        for(auto typeExpected : typeVectorTwo)
            if(ty == typeExpected){
                typeValid = true;
                break;
            }
        EXPECT_EQ(typeValid, true);
    }
}

#endif //ENV_MODEL_TEST_LANELET_H
