#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/on_same_road_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

bool OnSameRoadPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters) {
    std::unordered_set<unsigned long> relevantIDs;
    // we check here for state as it larger scenarios/maps, obstacle might be at end of road and if state is not
    // part of road ccs computations might fail (shape alone state might not be part of road)
    auto laneletsP = obstacleP->getOccupiedLaneletsByState(world->getRoadNetwork(), timeStep);
    auto refK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
    for (const auto &la : refK->getContainedLanelets()) {
        for (const auto &la2 : lanelet_operations::adjacentLanelets(la, false)) {
            if (std::any_of(laneletsP.begin(), laneletsP.end(),
                            [la2](const std::shared_ptr<Lanelet> &la) { return la->getId() == la2->getId(); }))
                return true;
        }
    }
    return false;
}

double OnSameRoadPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("OnSameRoadPredicate does not support robust evaluation!");
}

Constraint OnSameRoadPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("OnSameRoadPredicate does not support constraint evaluation!");
}
OnSameRoadPredicate::OnSameRoadPredicate() : CommonRoadPredicate(true) {}
