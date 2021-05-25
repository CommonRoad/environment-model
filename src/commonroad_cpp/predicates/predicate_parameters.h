//
// Created by Sebastian Maierhofer on 24.05.21.
//

#ifndef ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_
#define ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_

#include <cassert>

struct PredicateParameters {
    PredicateParameters() { checkParameterValidity(); }
    double aAbrupt{-2};
    double jAbrupt{-2}; // used for go vehicle emergency profile calculation
    double standstillError{0.01};
    double minVelocityDif{15.0};
    uint8_t numVehCongestion{3};
    double maxCongestionVelocity{2.78};
    uint8_t numVehSlowMovingTraffic{3};
    double maxSlowMovingTrafficVelocity{8.33};
    uint8_t numVehQueueOfVehicles{3};
    double maxQueueOfVehiclesVelocity{3};
    double maxVelocityLimitFreeDriving{16.67};
    double desiredInterstateVelocity{36.11};
    double minInterstateWidth{7.0};
    double closeToLaneBorder{7.0};
    double closeToOtherVehicle{0.5};
    double slightlyHigherSpeedDifference{5.55};
    double uTurn{1.57};
    double aboveCenterlineTh{0.1};

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
