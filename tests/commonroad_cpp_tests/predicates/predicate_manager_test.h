#pragma once

#include <gtest/gtest.h>

class PredicateManagerTest : public testing::Test {
  protected:
    void extractRelevantPredicatesHelper(int num_threads) const;
};
