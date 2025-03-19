#pragma once

#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h"
#include <gtest/gtest.h>

class TrafficSignElementTest : public testing::Test {
  protected:
    std::shared_ptr<TrafficSignElement> element1;
    std::shared_ptr<TrafficSignElement> element2;

  private:
    void SetUp() override;
};
