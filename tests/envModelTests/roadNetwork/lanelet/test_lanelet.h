//
// Created by sebastian on 03.12.20.
//

#ifndef ENV_MODEL_TEST_LANELET_H
#define ENV_MODEL_TEST_LANELET_H


#include "roadNetwork/lanelet/lanelet.h"
#include "../test_road_network.h"


class LaneletTest : public::RoadNetworkTest{
private:
    void SetUp() override;
};

#endif //ENV_MODEL_TEST_LANELET_H
