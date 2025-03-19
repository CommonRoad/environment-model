#pragma once

#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h"
#include <gtest/gtest.h>

class TrafficSignTest : public testing::Test {
  protected:
    std::shared_ptr<TrafficSign> sign1;
    std::shared_ptr<TrafficSign> sign2;

  private:
    void SetUp() override;
};
