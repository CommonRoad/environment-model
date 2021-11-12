//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <climits>
#include <cstddef>

#include "types_and_definitions.h"

struct TrafficLightCycleElement {
    TrafficLightState color;
    size_t duration;
};

struct vertex {
    double x;
    double y;
};

struct ValidStates {
    bool xPosition;
    bool yPosition;
    bool velocity;
    bool acceleration;
    bool lonPosition;
    bool latPosition;
    bool globalOrientation;
    bool curvilinearOrientation;
};

struct PredicateStatistics {
    long maxComputationTime{LONG_MIN};
    long minComputationTime{LONG_MAX};
    long totalComputationTime{0};
    long numExecutions{0};
    long numSatisfaction{0};
};

struct Constraint {
    double realValuedConstraint;
};

struct PredicateSatisfaction {
    PredicateSatisfaction(double costs, double satisfaction_probability)
        : costs(costs), satisfactionProbability(satisfaction_probability) {}
    double costs{100.0};
    double satisfactionProbability{0.0};
};