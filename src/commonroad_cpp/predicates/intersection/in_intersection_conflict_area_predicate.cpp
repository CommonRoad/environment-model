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

    const auto occupiedLaneletsK = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &letP : laneletsP) {
        for (const auto &letK : occupiedLaneletsK) {
            if (!letK->hasLaneletType(LaneletType::intersection))
                continue;
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

bool InIntersectionConflictAreaPredicate::evaluateSetBased(const size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const bool setBased, const bool must) {
    const bool occupancyK{evaluatesOccupancy(obstacleK, timeStep, setBased)};
    const bool occupancyP{evaluatesOccupancy(obstacleP, timeStep, setBased)};
    if (!occupancyK and !occupancyP)
        return InIntersectionConflictAreaPredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP, {"0.0"},
                                                                      setBased);

    // Set-based prediction of the kth vehicle: the exclusion of lanelets in the driving direction of the kth vehicle is
    // negated within the predicate, and set-based predictions contain no orientation. So no lanelet is guaranteed to be
    // occupied against the driving direction (must), and every occupied lanelet may be (may).
    if (occupancyK and must)
        return false;

    // Lanelets of the pth vehicle; for a set-based prediction, each covered behavior drives on one of the occupied
    // lanes: must: lanelets of all occupied lanes; may: lanelets of any occupied lane.
    std::vector<std::shared_ptr<Lanelet>> laneletsP;
    if (occupancyP) {
        const auto lanesP{obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, true)};
        for (const auto &lane : lanesP)
            for (const auto &let : lane->getContainedLanelets())
                if (std::none_of(
                        laneletsP.begin(), laneletsP.end(),
                        [&let](const std::shared_ptr<Lanelet> &other) { return other->getId() == let->getId(); }) and
                    (!must or std::all_of(lanesP.begin(), lanesP.end(), [&let](const std::shared_ptr<Lane> &other) {
                        const auto &lets{other->getContainedLanelets()};
                        return std::any_of(lets.begin(), lets.end(), [&let](const std::shared_ptr<Lanelet> &otherLet) {
                            return otherLet->getId() == let->getId();
                        });
                    })))
                    laneletsP.push_back(let);
    } else
        laneletsP = obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getContainedLanelets();

    const auto simLaneletsK{
        occupancyK ? std::vector<std::shared_ptr<Lanelet>>{}
                   : obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, false)};
    const auto occupiedLaneletsK{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &letP : laneletsP) {
        for (const auto &letK : occupiedLaneletsK) {
            if (!letK->hasLaneletType(LaneletType::intersection) or letK->getId() != letP->getId())
                continue;
            if (occupancyK) // may-version, see above
                return true;
            if (!std::any_of(simLaneletsK.begin(), simLaneletsK.end(),
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
