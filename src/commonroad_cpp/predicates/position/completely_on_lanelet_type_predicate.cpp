#include "commonroad_cpp/predicates/position/completely_on_lanelet_type_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <algorithm>

namespace bg = boost::geometry;

CompletelyOnLaneletTypePredicate::CompletelyOnLaneletTypePredicate() : CommonRoadPredicate(false) {}

bool CompletelyOnLaneletTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    // does not check lanelet driving direction
    // current implementation depends obstacle shape
    auto lanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    if (std::all_of(lanelets.begin(), lanelets.end(),
                    [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                        return lanelet->hasLaneletType(
                            lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0)));
                    }))
        return true;
    auto vertices = obstacleK->getOccupancyPolygonShape(timeStep).outer();
    for (auto vertice : vertices) {
        polygon_type polygonPos;
        bg::append(polygonPos, point_type{vertice.x(), vertice.y()});
        if (!std::any_of(lanelets.begin(), lanelets.end(),
                         [additionalFunctionParameters, polygonPos](const std::shared_ptr<Lanelet> &lanelet) {
                             return lanelet->hasLaneletType(lanelet_operations::matchStringToLaneletType(
                                        additionalFunctionParameters.at(0))) and
                                    lanelet->applyIntersectionTesting(polygonPos);
                         }))
            return false;
    }
    return true;
}

double CompletelyOnLaneletTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CompletelyOnOneLaneletTypePredicate does not support robust evaluation!");
}

Constraint CompletelyOnLaneletTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CompletelyOnOneLaneletTypePredicate does not support constraint evaluation!");
}
