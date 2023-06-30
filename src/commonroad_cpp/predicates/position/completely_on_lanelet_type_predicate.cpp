#include "commonroad_cpp/predicates/position/completely_on_lanelet_type_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <algorithm>

CompletelyOnLaneletTypePredicate::CompletelyOnLaneletTypePredicate() : CommonRoadPredicate(false) {}

bool CompletelyOnLaneletTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto vertices = obstacleK->getOccupancyPolygonShape(timeStep).outer();
    for (int i = 0; i < vertices.size(); i++) {
        std::vector<std::shared_ptr<Lanelet>> lanelets =
            world->getRoadNetwork()->findLaneletsByPosition(vertices[i].x(), vertices[i].y());
        if (std::all_of(lanelets.begin(), lanelets.end(),
                        [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                            return !lanelet->hasLaneletType(additionalFunctionParameters->laneletType[0]);
                        }))
            return false;
    }
    return true;
}

double CompletelyOnLaneletTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("CompletelyOnOneLaneletTypePredicate does not support robust evaluation!");
}

Constraint CompletelyOnLaneletTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("CompletelyOnOneLaneletTypePredicate does not support constraint evaluation!");
}
