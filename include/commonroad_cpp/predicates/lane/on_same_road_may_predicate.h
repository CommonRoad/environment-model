#pragma once

#include "commonroad_cpp/predicates/lane/on_same_road_predicate.h"

/**
 * May-version of on_same_road for a set-based prediction of the pth vehicle: at least one behavior covered by the
 * occupancy is on the road of the kth vehicle, i.e., a lanelet occupied by the occupancy is adjacent to a lanelet of
 * the reference lane of the kth vehicle. This is the set-based evaluation of on_same_road.
 */
class OnSameRoadMayPredicate : public OnSameRoadPredicate {};
