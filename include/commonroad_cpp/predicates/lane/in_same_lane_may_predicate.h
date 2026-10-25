#pragma once

#include "commonroad_cpp/predicates/lane/in_same_lane_predicate.h"

/**
 * May-version of in_same_lane for a set-based prediction of the pth vehicle: at least one behavior covered by the
 * occupancy shares a lane with the kth vehicle, i.e., a lanelet occupied by the occupancy belongs to a lane of the kth
 * vehicle. This is the set-based evaluation of in_same_lane.
 */
class InSameLaneMayPredicate : public InSameLanePredicate {};
