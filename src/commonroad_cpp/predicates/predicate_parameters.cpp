//
// Created by Sebastian Maierhofer on 25.05.21.
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
