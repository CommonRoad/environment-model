#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_leftmost_lane_predicate.h>
#include <commonroad_cpp/world.h>

bool InLeftmostLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(lanelets.begin(), lanelets.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
        return lanelet->getAdjacentLeft().adj == nullptr ||
               lanelet->getAdjacentLeft().oppositeDir != lanelet->getAdjacentLeft().adj->getAdjacentRight().oppositeDir;
    });
}

double InLeftmostLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("In Leftmost Lane Predicate does not support robust evaluation!");
}

Constraint InLeftmostLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("In Leftmost Lane Predicate does not support constraint evaluation!");
}

InLeftmostLanePredicate::InLeftmostLanePredicate() : CommonRoadPredicate(false) {}
