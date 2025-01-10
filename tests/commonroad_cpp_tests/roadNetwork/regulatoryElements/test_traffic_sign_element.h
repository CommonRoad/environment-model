//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
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
