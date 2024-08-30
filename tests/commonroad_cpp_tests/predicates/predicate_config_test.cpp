#include "predicate_config_test.h"

void PredicateConfigTest::SetUp() {}

TEST_F(PredicateConfigTest, updateParam) {
    EXPECT_EQ(params.getParam("laneMatchingOrientation"), 0.35);
    params.updateParam("laneMatchingOrientation", 0.0);
    EXPECT_EQ(params.getParam("laneMatchingOrientation"), 0.0);
}

TEST_F(PredicateConfigTest, getPredicateNames) {
    EXPECT_EQ(params.getConstantNames().at(0), "brakingSpeedLimit");
    EXPECT_EQ(params.getPredicateNames().at(0), "aBrakingIntersection");
}

TEST_F(PredicateConfigTest, getParameterCollection) {
    EXPECT_EQ(std::get<0>(params.getParameterCollection().at("laneMatchingOrientation")), "laneMatchingOrientation");
    EXPECT_EQ(std::get<3>(params.getParameterCollection().at("laneMatchingOrientation")), 0.0);
    EXPECT_EQ(std::get<5>(params.getParameterCollection().at("laneMatchingOrientation")), "angle");
}
