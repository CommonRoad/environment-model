//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "in_intersection_conflict_area_predicate.h"
#include "../../obstacle/obstacle.h"
#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lane.h"
#include "../../world.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "on_lanelet_with_type_predicate.h"
#include "in_front_of_predicate.h"

bool InIntersectionConflictAreaPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    OnLaneletWithTypePredicate onLaneletWithTypePredicate;
    std::shared_ptr<OptionalPredicateParameters> opt{
        std::make_shared<OptionalPredicateParameters>(std::vector<LaneletType>{LaneletType::intersection})};
    if (!onLaneletWithTypePredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP, opt))
        return false;
    auto simLaneletsK{obstacle_operations::getSimilarlyOrientedLanelets(
        world->getRoadNetwork(), obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        obstacleK->getStateByTimeStep(timeStep), parameters.laneletOccupancySimilarity)};
    auto simLaneletsP{obstacle_operations::getSimilarlyOrientedLanelets(
        world->getRoadNetwork(), obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        obstacleP->getStateByTimeStep(timeStep), parameters.laneletOccupancySimilarity)};
    std::vector<std::shared_ptr<Lane>> lanes;
    for (const auto &simLaneletP : simLaneletsP)
        for (const auto &lane : world->getRoadNetwork()->findLanesByContainedLanelet(simLaneletP->getId()))
            lanes.push_back(lane);
    InFrontOfPredicate inFrontOfPredicate;
    for (const auto &lane : lanes) {
        if (!lane->getCurvilinearCoordinateSystem()->cartesianPointInProjectionDomain(
                obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                obstacleK->getStateByTimeStep(timeStep)->getYPosition()) or
            !inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP))
            continue;
        for (const auto &letP : lane->getContainedLanelets()) {
            if (!letP->hasLaneletType(LaneletType::intersection))
                continue;
            for (const auto &letK : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep))
                if (letK->getId() == letP->getId() and !std::any_of(simLaneletsK.begin(), simLaneletsK.end(),
                                                                    [letK](const std::shared_ptr<Lanelet> &letSim) {
                                                                        return letSim->getId() == letK->getId();
                                                                    }))
                    return true;
        }
    }
    return false;
}

double InIntersectionConflictAreaPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionConflictAreaPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support constraint evaluation!");
}

InIntersectionConflictAreaPredicate::InIntersectionConflictAreaPredicate() : CommonRoadPredicate(true) {}