//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/predicates/predicate_config.h>

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
    assert(uTurnUpper > 0);
    assert(uTurnLower > 0);
    assert(aboveCenterlineTh > 0);
}

std::map<std::string, std::array<double, 2>> predicateSatisfaction{
    {"keeps_safe_distance_prec", {95.0, 0.85}},
    {"unnecessary_braking", {90.0, 0.05}},
    {"in_same_lane", {1.0, 0.15}},
    {"in_front_of", {85.0, 0.45}},
    {"orientation_towards", {95.0, 0.01}},
    {"in_single_lane", {5.0, 80.0}},
    {"keeps_lane_speed_limit", {1.0, 0.80}},
    {"keeps_sign_min_speed_limit", {1.0, 0.95}},
    {"lane_based_orientation_similar", {10.0, 0.99}},
    {"preserves_traffic_flow", {30.0, 0.70}},
    {"slow_leading_vehicle", {99.0, 0.10}},
};
