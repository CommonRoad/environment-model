#pragma once

#include <gtest/gtest.h>

class GeometricOperationsTest : public testing::Test {
    void SetUp() override;

  protected:
    double epsilon{0.000001};
};
