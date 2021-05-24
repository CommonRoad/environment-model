//
// Created by Sebastian Maierhofer on 24.05.21.
//

#ifndef ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_
#define ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_

#include <cassert>

struct PredicateParameters {
  PredicateParameters(){
    checkParameterValidity();
  }
  double aAbrupt{-2};

  bool checkParameterValidity() const {
    assert(aAbrupt < 0);
  }
};

#endif //ENV_MODEL_SRC_COMMONROAD_CPP_PREDICATES_PREDICATE_PARAMETERS_H_
