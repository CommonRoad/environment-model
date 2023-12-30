//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "utility_functions.h"
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <gtest/gtest.h>

class InterfacesTest : public testing::Test {

  protected:
    /**
     * Loads two scenarios from same file name with xml and protobuf format.
     *
     * @param name File name
     * @return Xml and protobuf scenario
     */
    static std::tuple<Scenario, Scenario> loadXmlAndPbScenarios(const std::string &name);
};
