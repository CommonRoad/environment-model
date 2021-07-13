//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "predicate_parameters.h"

void PredicateParameters::checkParameterValidity() const {
    assert(aAbrupt < 0);
    assert(jAbrupt < 0);
    assert(standstillError > 0);
    assert(minVelocityDif > 0);
    assert(numVehCongestion > 0);
    assert(maxCongestionVelocity > 0);
    assert(numVehSlowMovingTraffic > 0);
    assert(maxSlowMovingTrafficVelocity > 0);
    assert(numVehQueueOfVehicles > 0);
    assert(maxQueueOfVehiclesVelocity > 0);
    assert(maxVelocityLimitFreeDriving > 0);
    assert(desiredInterstateVelocity > 0);
    assert(minInterstateWidth > 0);
    assert(closeToLaneBorder > 0);
    assert(closeToOtherVehicle > 0);
    assert(slightlyHigherSpeedDifference > 0);
    assert(uTurn > 0);
    assert(aboveCenterlineTh > 0);
}
