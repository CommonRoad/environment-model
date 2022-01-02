//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <gtest/gtest.h>

class PredicateManagerTest : public testing::Test {
  protected:
    void extractRelevantPredicatesHelper(int num_threads) const;
};
