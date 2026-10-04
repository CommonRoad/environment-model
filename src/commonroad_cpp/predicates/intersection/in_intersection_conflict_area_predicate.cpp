#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool InIntersectionConflictAreaPredicate::booleanEvaluation(
    const size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    const bool setBased) {

    auto simLaneletsK{
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    std::vector<std::shared_ptr<Lanelet>> laneletsP;
    if (setBased and
        !obstacleP->getSetBasedPrediction().empty()) // we do not check for initial state as we compute ref lane
        for (const auto &lane : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased))
            laneletsP.insert(laneletsP.end(), lane->getContainedLanelets().begin(), lane->getContainedLanelets().end());
    else {
        const auto lane{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
        laneletsP = lane->getContainedLanelets();
    }

    // For a set-based prediction of the kth vehicle, the predicate is used as may-version (it appears negated in
    // R_IN5 via not_endanger_intersection). The exclusion of similarly oriented lanelets is negated within the
    // predicate and would require lanelets occupied by all covered behaviors; since set-based predictions contain no
    // orientation, the exclusion is skipped, which over-approximates the predicate.
    const bool usesOccupancyK{setBased and !obstacleK->getSetBasedPrediction().empty() and
                              timeStep > obstacleK->getCurrentState()->getTimeStep()};
    const auto occupiedLaneletsK = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &letP : laneletsP) {
        for (const auto &letK : occupiedLaneletsK) {
            if (!letK->hasLaneletType(LaneletType::intersection))
                continue;
            if (usesOccupancyK and letK->getId() == letP->getId())
                return true;
            if (letK->getId() == letP->getId() and
                !std::any_of(simLaneletsK.begin(), simLaneletsK.end(),
                             [letK](const std::shared_ptr<Lanelet> &letSim) {
                                 return letSim->hasLaneletType(LaneletType::intersection) and
                                        letSim->getId() == letK->getId();
                             }) and
                !std::any_of(simLaneletsK.begin(), simLaneletsK.end(), [letP](const std::shared_ptr<Lanelet> &letSim) {
                    return letSim->hasLaneletType(LaneletType::intersection) and
                           intersection_operations::checkSameIncoming(
                               letP, letSim, SensorParameters::dynamicDefaults().getFieldOfViewFront(),
                               1); // extend only until next intersection
                }))
                return true;
        }
    }

    return false;
}

double InIntersectionConflictAreaPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionConflictAreaPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support constraint evaluation!");
}

InIntersectionConflictAreaPredicate::InIntersectionConflictAreaPredicate() : CommonRoadPredicate(true) {}
