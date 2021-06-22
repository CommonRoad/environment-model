//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <memory>
#include <vector>
#include <unordered_map>

#include "types_and_definitions.h"
#include "traffic_signs.h"

using TrafficSignTable = const std::unordered_map<TrafficSignTypes, std::string>;

extern TrafficSignTable TrafficSignIDGermany;
extern TrafficSignTable TrafficSignIDZamunda;
extern TrafficSignTable TrafficSignIDUSA;
extern TrafficSignTable TrafficSignIDSpain;

extern const std::unordered_map<SupportedTrafficSignCountry,
    TrafficSignTable *>
    TrafficSignLookupTableByCountry;

extern const std::unordered_map<std::string, std::vector<int>> priorityTable;
