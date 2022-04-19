//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "in_intersection_conflict_area_predicate.h"
#include "../../obstacle/obstacle.h"
#include "../../roadNetwork/lanelet/lane.h"
#include "../../world.h"
#include "on_lanelet_with_type_predicate.h"

bool InIntersectionConflictAreaPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    OnLaneletWithTypePredicate onLaneletWithTypePredicate;
    std::shared_ptr<OptionalPredicateParameters> opt{
        std::make_shared<OptionalPredicateParameters>(std::vector<LaneletType>{LaneletType::intersection})};
    if (!onLaneletWithTypePredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP, opt))
        return false;
    for (const auto &lane : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep))
        for (const auto &letP : lane->getContainedLanelets())
            for (const auto &letK : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep))
                if (letK->getId() != letP->getId() and
                    (letP->getAdjacentLeft().adj == nullptr or
                     letK->getId() != letP->getAdjacentLeft().adj->getId()) and
                    (letP->getAdjacentLeft().adj == nullptr or
                     letK->getId() != letP->getAdjacentRight().adj->getId()) and
                    !std::any_of(
                        letP->getPredecessors().begin(), letP->getPredecessors().end(),
                        [letK](const std::shared_ptr<Lanelet> &letPre) { return letK->getId() == letPre->getId(); }) and
                    !std::any_of(
                        letP->getSuccessors().begin(), letP->getSuccessors().end(),
                        [letK](const std::shared_ptr<Lanelet> &letSuc) { return letK->getId() == letSuc->getId(); }) and
                    letK->applyIntersectionTesting(letP->getOuterPolygon()))
                    return true;
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