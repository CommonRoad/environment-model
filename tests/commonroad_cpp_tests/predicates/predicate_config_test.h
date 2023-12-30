#pragma once

#include "commonroad_cpp/predicates/predicate_config.h"
#include <gtest/gtest.h>

class PredicateConfigTest : public testing::Test {
  protected:
    PredicateParameters params;

  private:
    void SetUp() override;
};
