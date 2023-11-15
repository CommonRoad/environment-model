#include "predicate_config_test.h"

void PredicateConfigTest::SetUp() {}

TEST_F(PredicateConfigTest, updateParam) {
    EXPECT_EQ(params.paramMap["dMinUrban"], 1.5);
    params.updateParam("dMinUrban", 0.0);
    EXPECT_EQ(params.paramMap["dMinUrban"], 0.0);
}