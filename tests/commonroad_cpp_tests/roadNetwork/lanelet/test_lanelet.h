#pragma once

#include <gtest/gtest.h>

#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

class LaneletTestInitialization {
  protected:
    size_t idLaneletOne;
    std::set<LaneletType> laneletTypeLaneletOne;
    std::set<ObstacleType> userOneWayLaneletOne;
    std::set<ObstacleType> userBidirectionalLaneletOne;
    std::vector<vertex> centerVerticesLaneletOne;
    std::vector<vertex> leftBorderLaneletOne;
    std::vector<vertex> rightBorderLaneletOne;
    std::shared_ptr<Lanelet> adjacentRight;
    std::shared_ptr<Lanelet> laneletOne;

    size_t idLaneletTwo;
    std::set<LaneletType> laneletTypeLaneletTwo;
    std::vector<vertex> centerVerticesLaneletTwo;
    std::vector<vertex> leftBorderLaneletTwo;
    std::vector<vertex> rightBorderLaneletTwo;
    std::shared_ptr<Lanelet> laneletTwo;
    std::set<ObstacleType> userOneWayLaneletTwo;
    std::set<ObstacleType> userBidirectionalLaneletTwo;

    size_t idLaneletThree;
    std::set<LaneletType> laneletTypeLaneletThree;
    std::vector<vertex> centerVerticesLaneletThree;
    std::vector<vertex> leftBorderLaneletThree;
    std::vector<vertex> rightBorderLaneletThree;
    std::shared_ptr<Lanelet> laneletThree;

    std::shared_ptr<Lanelet> laneletFour;
    std::shared_ptr<Lanelet> laneletFive;
    std::shared_ptr<Lanelet> laneletSix;
    std::shared_ptr<Lanelet> laneletSeven;

    polygon_type polygonOne;
    polygon_type polygonTwo;
    polygon_type polygonThree;

    static void compareVerticesVector(std::vector<vertex> verticesOne, std::vector<vertex> verticesTwo);
    template <typename T> void evaluateTypes(std::vector<T> typeVectorOne, std::vector<T> typeVectorTwo);

    void setUpLanelets();
};

class LaneletTest : public LaneletTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

template <typename T>
void LaneletTestInitialization::evaluateTypes(std::vector<T> typeVectorOne, std::vector<T> typeVectorTwo) {
    for (auto ty : typeVectorOne) {
        bool typeValid{false};
        for (auto typeExpected : typeVectorTwo)
            if (ty == typeExpected) {
                typeValid = true;
                break;
            }
        EXPECT_EQ(typeValid, true);
    }
}
