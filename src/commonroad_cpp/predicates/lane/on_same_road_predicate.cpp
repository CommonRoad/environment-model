#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/on_same_road_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

bool OnSameRoadPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    std::unordered_set<unsigned long> relevantIDs;
    // temporarily, we checked here for state as in larger scenarios/maps, obstacle might be at end of road and if state
    // is not part of road ccs computations might fail (when using shape alone, state might not be on road) we changed
    // it to shape again since this simplifies set-based eval and assume we perform pre-checks to prevent failing ccs in
    // trajectory eval
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, setBased);
    auto refK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep, setBased)};
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
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    throw std::runtime_error("OnSameRoadPredicate does not support robust evaluation!");
}

Constraint OnSameRoadPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("OnSameRoadPredicate does not support constraint evaluation!");
}
OnSameRoadPredicate::OnSameRoadPredicate() : CommonRoadPredicate(true) {}
