#pragma once

#include "test_incoming.h"

class IntersectionTest : public IntersectionTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
