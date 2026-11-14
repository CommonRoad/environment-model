#include <algorithm>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_same_lane_must_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <unordered_set>

bool InSameLaneMustPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters,
                                                bool setBased) {
    if (evaluatesOccupancy(obstacleK, timeStep, setBased))
        return false;
    if (!evaluatesOccupancy(obstacleP, timeStep, setBased))
        return InSameLanePredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP,
                                                      additionalFunctionParameters, setBased);
    // each covered behavior occupies some of the lanelets of the occupancy
    std::unordered_set<size_t> laneletIdsK;
    for (const auto &laneK : obstacleK->getOccupiedLanesDrivingDirection(world->getRoadNetwork(), timeStep)) {
        const auto &ids{laneK->getContainedLaneletIDs()};
        laneletIdsK.insert(ids.begin(), ids.end());
    }
    const auto laneletsP{obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, true)};
    return !laneletsP.empty() and
           std::all_of(laneletsP.begin(), laneletsP.end(), [&laneletIdsK](const std::shared_ptr<Lanelet> &laneletP) {
               return laneletIdsK.count(laneletP->getId()) == 1;
           });
}
