#pragma once

#include "commonroad_cpp/predicates/predicate_parameter_collection.h"
#include <gtest/gtest.h>

class PredicateConfigTest : public testing::Test {
  protected:
    PredicateParameters params;

  private:
    void SetUp() override;
};
