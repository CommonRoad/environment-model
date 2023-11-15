//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/predicates/predicate_config.h>
#include <stdexcept>

void PredicateParameters::checkParameterValidity() {
    assert(paramMap["aAbrupt"] < 0);
    assert(paramMap["jAbrupt"] < 0);
    assert(paramMap["standstillError"] > 0);
    assert(paramMap["minVelocityDif"] > 0);
    assert(paramMap["numVehCongestion"] > 0);
    assert(paramMap["maxCongestionVelocity"] > 0);
    assert(paramMap["numVehSlowMovingTraffic"] > 0);
    assert(paramMap["maxSlowMovingTrafficVelocity"] > 0);
    assert(paramMap["numVehQueueOfVehicles"] > 0);
    assert(paramMap["maxQueueOfVehiclesVelocity"] > 0);
    assert(paramMap["maxVelocityLimitFreeDriving"] > 0);
    assert(paramMap["desiredInterstateVelocity"] > 0);
    assert(paramMap["minInterstateWidth"] > 0);
    assert(paramMap["closeToLaneBorder"] > 0);
    assert(paramMap["closeToOtherVehicle"] > 0);
    assert(paramMap["slightlyHigherSpeedDifference"] > 0);
    assert(paramMap["uTurnUpper"] > 0);
    assert(paramMap["uTurnLower"] > 0);
    assert(paramMap["aboveCenterlineTh"] > 0);
}

void PredicateParameters::updateParam(const std::string &name, double value) {
    if (paramMap.count(name) == 1) {
        paramMap[name] = value;
        checkParameterValidity();
    } else
        throw std::runtime_error("No predicate " + name + " found for update");
}

double PredicateParameters::getParam(const std::string &name) {
    if (paramMap.count(name) == 1) {
        return paramMap[name];
    } else
        throw std::runtime_error("No predicate " + name + " found");
}

std::map<std::string, std::array<double, 2>> predicateSatisfaction{
    {"keeps_safe_distance_prec", {95.0, 0.85}},
    {"unnecessary_braking", {90.0, 0.05}},
    {"in_same_lane", {1.0, 0.15}},
    {"in_front_of", {85.0, 0.45}},
    {"orientation_towards", {95.0, 0.01}},
    {"in_single_lane", {5.0, 0.80}},
    {"keeps_lane_speed_limit", {1.0, 0.80}},
    {"keeps_sign_min_speed_limit", {1.0, 0.95}},
    {"lane_based_orientation_similar", {10.0, 0.99}},
    {"preserves_traffic_flow", {30.0, 0.70}},
    {"slow_leading_vehicle", {99.0, 0.10}},
};
