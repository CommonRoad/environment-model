//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

// Note: This include needs to come first in order to have M_PI defined
// See https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include <commonroad_cpp/auxiliaryDefs/structs.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

struct PredicateParameters {
    PredicateParameters() { checkParameterValidity(); }
    double aAbrupt{-2};          // acceleration difference which indicates abrupt braking
    double jAbrupt{-2};          // used for go vehicle emergency profile calculation
    double standstillError{0.1}; // velocity deviation from zero which is still classified to be standstill
    double minVelocityDif{15.0}; // minimum velocity difference
    uint8_t numVehCongestion{
        3}; // minimum number of leading vehicles so that a vehicle can be assumed to be part of a congestion
    double maxCongestionVelocity{2.78}; // maximum velocity of a vehicle withing a congestion
    uint8_t numVehSlowMovingTraffic{
        3}; // minimum number of leading vehicles so that a vehicle can be assumed to be part of slow-moving traffic
    double maxSlowMovingTrafficVelocity{8.33}; // maximum velocity of a slow-moving vehicle
    uint8_t numVehQueueOfVehicles{
        3}; // minimum number of leading vehicles so that a vehicle can be assumed to be part of a queue of vehicles
    double maxQueueOfVehiclesVelocity{3};       // maximum velocity of a vehicle withing a queue of vehicles
    double maxVelocityLimitFreeDriving{16.67};  // maximum velocity of a free-driving vehicle
    double minInterstateWidth{7.0};             // minimum interstate width so that emergency lane can be created
    double closeToLaneBorder{0.2};              // indicator if vehicle is close to lane border
    double closeToOtherVehicle{0.5};            // indicator if vehicle is close to another vehicle
    double slightlyHigherSpeedDifference{5.55}; // indicator for slightly higher speed
    double uTurnLower{0.25 * M_PI};             // lower angle [rad] indicating u-turn on interstates
    double uTurnUpper{0.75 * M_PI};             // upper angle [rad] indicating u-turn on interstates
    double aboveCenterlineTh{0.1};              // indicator for necessary distance to be classified above centerline
    double epsilon{1e-6};                       // small value close to zero for different purposes
    double stopLineDistance{1.0};               // maximum distance vehicle waiting in front of stop line has to wait
    double laneMatchingOrientation{0.35};       // orientation threshold for following a lane
    double minSafetyDistance{5.0};              // minimum safety distance between two vehicles
    double closeToBicycle{6.0};                 // indicator if vehicle is close to a bicycle
    double narrowRoad{5.5};                     // maximum width of road to be called narrow

    void checkParameterValidity() const;

    double maxPositiveDouble{
        std::numeric_limits<double>::max()}; // max double value close to zero for different purposes
    double desiredInterstateVelocity{36.11}; // desired velocity on interstates

    double fovSpeedLimit{50}; //**< compute with calc_v_max_fov(ego_vehicle_param, simulation_param) */
    double brakingSpeedLimit{
        43}; //**< compute with calc_v_max_braking(ego_vehicle_param, simulation_param, traffic_rule_param) */
    double roadConditionSpeedLimit{50};

    double dBrakingIntersection{15.0};
    double aBrakingIntersection{-1.0};

    double dMinUrban{1.5};
    double dMinNonUrban{2.0};

    double laneletOccupancySimilarity{0.25};

    double intersectionBrakingPossible{-4.0};

    double close_to_intersection_max_distance{30};
};

extern std::map<std::string, std::array<double, 2>> predicateSatisfaction;
