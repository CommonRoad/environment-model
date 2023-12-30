//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/world.h"

#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/position/left_of_broad_lane_marking_predicate.h>

bool LeftOfBroadLaneMarkingPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets_occ =
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);

    for (auto &lanelet : lanelets_occ) {
        const LineMarking lanelet_left_marking = lanelet->getLineMarkingLeft();
        if (lanelet_left_marking == LineMarking::broad_dashed or lanelet_left_marking == LineMarking::broad_solid)
            return false;
    }

    std::set<std::shared_ptr<Lanelet>> lanelets_right_of_veh =
        obstacle_operations::laneletsRightOfObstacle(timeStep, world->getRoadNetwork(), obstacleK);
    return std::any_of(lanelets_right_of_veh.begin(), lanelets_right_of_veh.end(),
                       [](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet->getLineMarkingLeft() == LineMarking::broad_dashed or
                                  lanelet->getLineMarkingLeft() == LineMarking::broad_solid;
                       });
}

double LeftOfBroadLaneMarkingPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Left Of Broad Lane Marking Predicate does not support robust evaluation!");
}

Constraint LeftOfBroadLaneMarkingPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Left Of Broad Lane Marking Predicate does not support constraint evaluation!");
}

LeftOfBroadLaneMarkingPredicate::LeftOfBroadLaneMarkingPredicate() : CommonRoadPredicate(false) {}