#pragma once

#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"

/**
 * Must-version of in_front_of for a set-based prediction of the kth vehicle: all behaviors covered by the occupancy
 * are in front of the pth vehicle, i.e., the rear of the occupancy is in front of the front of the pth vehicle. This is
 * the set-based evaluation of in_front_of.
 */
class InFrontOfMustPredicate : public InFrontOfPredicate {};
