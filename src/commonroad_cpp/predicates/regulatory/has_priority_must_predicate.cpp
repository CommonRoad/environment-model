#include <algorithm>
#include <limits>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/regulatory/has_priority_must_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/world.h>

namespace {
/**
 * Priority bound of a vehicle over all behaviors covered by its occupancy.
 *
 * @param lanelets Lanelets occupied by the occupancy.
 * @param dir Direction of the vehicle.
 * @param lowest Whether the lowest (true) or the highest (false) priority of the single lanelets is returned.
 * @return Priority bound; the lowest int if a covered behavior may occupy no incoming lanelet.
 */
int priorityBound(const std::vector<std::shared_ptr<Lanelet>> &lanelets, const Direction dir, const bool lowest) {
    int bound{std::numeric_limits<int>::min()};
    for (const auto &let : lanelets) {
        if (!let->hasLaneletType(LaneletType::incoming))
            return std::numeric_limits<int>::min();
        const int prio{regulatory_elements_utils::extractPriorityTrafficSign({let}, dir)};
        if (prio == std::numeric_limits<int>::min())
            return std::numeric_limits<int>::min();
        if (bound == std::numeric_limits<int>::min() or (lowest ? prio < bound : prio > bound))
            bound = prio;
    }
    return bound;
}
} // namespace

bool HasPriorityMustPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    const bool occupancyK{evaluatesOccupancy(obstacleK, timeStep, setBased)};
    const bool occupancyP{evaluatesOccupancy(obstacleP, timeStep, setBased)};
    if (!occupancyK and !occupancyP)
        return HasPriorityPredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP,
                                                       additionalFunctionParameters, setBased);
    const auto dirK{regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0))};
    const auto dirP{regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(1))};
    // the kth vehicle has priority for all covered behaviors if its lowest priority exceeds the highest priority of the
    // pth vehicle
    const int prioK{
        occupancyK
            ? priorityBound(obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, true), dirK, true)
            : regulatory_elements_utils::getPriority(timeStep, world->getRoadNetwork(), obstacleK, dirK)};
    const int prioP{
        occupancyP
            ? priorityBound(obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, true), dirP, false)
            : regulatory_elements_utils::getPriority(timeStep, world->getRoadNetwork(), obstacleP, dirP)};
    return prioK > prioP and prioK != std::numeric_limits<int>::min() and prioP != std::numeric_limits<int>::min();
}
