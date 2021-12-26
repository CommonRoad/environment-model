//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <gtest/gtest.h>

class GeometricOperationsTest : public testing::Test {
  private:
    void SetUp() override;

  protected:
    double epsilon{0.000001};
};
