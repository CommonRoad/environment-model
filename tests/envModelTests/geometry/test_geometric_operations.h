//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_GEOMETRIC_OPERATIONS_H
#define ENV_MODEL_TEST_GEOMETRIC_OPERATIONS_H

#include <gtest/gtest.h>
#include "geometry/geometric_operations.h"

class GeometricOperationsTest : public testing::Test {
private:
    void SetUp() override;

protected:
    double epsilon { 0.000001 };
};


#endif //ENV_MODEL_TEST_GEOMETRIC_OPERATIONS_H
