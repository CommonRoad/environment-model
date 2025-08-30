#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/intersection/at_intersection_type_predicate.h>

bool AtIntersectionTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    return world->getRoadNetwork()
        ->getIntersectionByID(std::stoul(additionalFunctionParameters[1]))
        ->hasIntersectionType(
            intersection_operations::matchStringToIntersectionType(additionalFunctionParameters.at(0)));
}

double AtIntersectionTypePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("AtIntersectionTypePredicate does not support robust evaluation!");
}
Constraint AtIntersectionTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("AtIntersectionTypePredicate does not support constraint evaluation!");
}
AtIntersectionTypePredicate::AtIntersectionTypePredicate() : CommonRoadPredicate(false) {}
