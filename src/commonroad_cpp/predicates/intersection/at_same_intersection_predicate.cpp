#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/at_same_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>

bool AtSameIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {

    const auto currentIntersection_k = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    if (const auto currentIntersection_p = intersection_operations::currentIntersection(timeStep, world, obstacleP);
        currentIntersection_k != nullptr && currentIntersection_p != nullptr) {
        return currentIntersection_k->getId() == currentIntersection_p->getId();
    }

    return false;
}
double AtSameIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("AtSameIntersectionPredicate does not support robust evaluation!");
}
Constraint AtSameIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("AtSameIntersectionPredicate does not support constraint evaluation!");
}
AtSameIntersectionPredicate::AtSameIntersectionPredicate() : CommonRoadPredicate(true) {}
