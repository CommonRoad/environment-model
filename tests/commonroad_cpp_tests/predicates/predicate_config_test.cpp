#include "predicate_config_test.h"

void PredicateConfigTest::SetUp() {}

TEST_F(PredicateConfigTest, updateParam) {
    EXPECT_EQ(params.getParam("dMinUrban"), 1.5);
    params.updateParam("dMinUrban", 0.0);
    EXPECT_EQ(params.getParam("dMinUrban"), 0.0);
}