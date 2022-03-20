//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include <gtest/gtest.h>

class TrafficLightTest : public testing::Test {
  protected:
    std::shared_ptr<TrafficLight> light1;
    std::shared_ptr<TrafficLight> light2;

  private:
    void SetUp() override;
};
