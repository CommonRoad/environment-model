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

    std::map<std::string, double> paramMap{
        {"aAbrupt", -2.0},          // acceleration difference which indicates abrupt braking
        {"jAbrupt",-2.0},          // used for go vehicle emergency profile calculation
        {"standstillError", 0.1}, // velocity deviation from zero which is still classified to be standstill
        {"minVelocityDif", 15.0}, // minimum velocity difference
        {"numVehCongestion", 3.0}, // minimum number of leading vehicles so that a vehicle can be assumed to be part of a congestion
        {"maxCongestionVelocity", 2.78}, // maximum velocity of a vehicle withing a congestion
        {"numVehSlowMovingTraffic", 3.0}, // minimum number of leading vehicles so that a vehicle can be assumed to be part of slow-moving traffic
        {"maxSlowMovingTrafficVelocity", 8.33}, // maximum velocity of a slow-moving vehicle
        {"numVehQueueOfVehicles", 3.0}, // minimum number of leading vehicles so that a vehicle can be assumed to be part of a queue of vehicles
        {"maxQueueOfVehiclesVelocity", 3},       // maximum velocity of a vehicle withing a queue of vehicles
        {"maxVelocityLimitFreeDriving", 16.67},  // maximum velocity of a free-driving vehicle
        {"minInterstateWidth", 7.0},             // minimum interstate width so that emergency lane can be created
        {"closeToLaneBorder", 0.2},              // indicator if vehicle is close to lane border
        {"closeToOtherVehicle", 0.5},            // indicator if vehicle is close to another vehicle
        {"slightlyHigherSpeedDifference", 5.55}, // indicator for slightly higher speed
        {"uTurnLower", 0.25 * M_PI},           // lower angle [rad] indicating u-turn on interstates
        {"uTurnUpper", 0.75 * M_PI},             // upper angle [rad] indicating u-turn on interstates
        {"aboveCenterlineTh", 0.1},              // indicator for necessary distance to be classified above centerline
        {"epsilon", 1e-6},                       // small value close to zero for different purposes
        {"stopLineDistance", 1.0},               // maximum distance vehicle waiting in front of stop line has to wait
        {"laneMatchingOrientation", 0.35},       // orientation threshold for following a lane
        {"minSafetyDistance", 5.0},             // minimum safety distance between two vehicles
        {"closeToBicycle",6.0},                 // indicator if vehicle is close to a bicycle
        {"narrowRoad", 5.5},                     // maximum width of road to be called narrow
        {"maxPositiveDouble", std::numeric_limits<double>::max()}, // max double value close to zero for different purposes
        {"desiredInterstateVelocity", 36.11}, // desired velocity on interstates

        {"fovSpeedLimit", 50}, //**< compute with calc_v_max_fov(ego_vehicle_param, simulation_param) */
        {"brakingSpeedLimit", 43}, //**< compute with calc_v_max_braking(ego_vehicle_param, simulation_param, traffic_rule_param) */
        {"roadConditionSpeedLimit", 50},

        {"dBrakingIntersection", 15.0},
        {"aBrakingIntersection", -1.0},

        {"dMinUrban",1.5},
        {"dMinNonUrban",2.0},

        {"laneletOccupancySimilarity", 0.25},

        {"intersectionBrakingPossible", -4.0},

        {"close_to_intersection_max_distance",30},
    };

    void checkParameterValidity();

    /**
     * Updates single predicate parameter
     *
     * @param name (string) name of parameter
     * @param value (double) value to be set
     */
    void updateParam(const std::string& name, double value);

    /**
     * Getter for predicate parameter
     *
     * @param name (string) name of parameter
     */
    double getParam(const std::string& name);


};

extern std::map<std::string, std::array<double, 2>> predicateSatisfaction;
