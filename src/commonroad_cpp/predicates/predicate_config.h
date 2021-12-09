//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "../auxiliaryDefs/structs.h"

#include <cassert>
#include <limits>
#include <map>
#include <string>

struct PredicateParameters {
    PredicateParameters() { checkParameterValidity(); }
    double aAbrupt{-2};           // acceleration difference which indicates abrupt braking
    double jAbrupt{-2};           // used for go vehicle emergency profile calculation
    double standstillError{0.01}; // velocity deviation from zero which is still classified to be standstill
    double minVelocityDif{15.0};  // minimum velocity difference
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
    double desiredInterstateVelocity{36.11};    // desired velocity on interstates
    double minInterstateWidth{7.0};             // minimum interstate width so that emergency lane can be created
    double closeToLaneBorder{7.0};              // indicator if vehicle is close to lane border
    double closeToOtherVehicle{0.5};            // indicator if vehicle is close to another vehicle
    double slightlyHigherSpeedDifference{5.55}; // indicator for slightly higher speed
    double uTurn{1.57};                         // angle [rad] indicating u-turn
    double aboveCenterlineTh{0.1};              // indicator for necessary distance to be classified above centerline
    double epsilon{1e-6};                       // small value close to zero for different purposes
    double maxPositiveDouble{
        std::numeric_limits<double>::max()}; // max double value close to zero for different purposes
    double stopLineDistance{1.0};            // maximum distance vehicle waiting in front of stop line has to wait
    double laneMatchingOrientation{0.35};    // orientation threshold for following a lane

    void checkParameterValidity() const;
};
