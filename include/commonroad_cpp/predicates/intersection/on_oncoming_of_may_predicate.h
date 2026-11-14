#pragma once

#include "commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.h"

/**
 * May-version of on_oncoming_of for a set-based prediction of the kth vehicle: at least one behavior covered by the
 * occupancy is on an oncoming lanelet of the pth vehicle, i.e., a lanelet occupied by the occupancy in its driving
 * direction is an oncoming lanelet. This is the set-based evaluation of on_oncoming_of. The lanelets in driving
 * direction of an occupancy are the occupied lanelets which belong to the lanes of the current time step, so covered
 * behaviors which change to an oncoming lanelet of another lane are not considered.
 */
class OnOncomingOfMayPredicate : public OnOncomingOfPredicate {};
