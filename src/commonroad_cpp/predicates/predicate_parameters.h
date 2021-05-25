//
// Created by Sebastian Maierhofer on 24.05.21.
//

#ifndef ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_
#define ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_

#include <cassert>

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

    void checkParameterValidity() const;
};

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

#endif // ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_
