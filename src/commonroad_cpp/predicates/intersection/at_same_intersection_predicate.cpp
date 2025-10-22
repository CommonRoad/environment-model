#include <algorithm>
#include <commonroad_cpp/predicates/intersection/at_same_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <stdexcept>

bool AtSameIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    const auto currentIntersection_k = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    const auto currentIntersection_p = intersection_operations::currentIntersection(timeStep, world, obstacleP);

    return std::find_if(currentIntersection_k.begin(), currentIntersection_k.end(),
                        [additionalFunctionParameters](const std::shared_ptr<Intersection> &intersection) {
                            return intersection->getId() == std::stoul(additionalFunctionParameters[0]);
                        }) != currentIntersection_k.end() &&
           std::find_if(currentIntersection_p.begin(), currentIntersection_p.end(),
                        [additionalFunctionParameters](const std::shared_ptr<Intersection> &intersection) {
                            return intersection->getId() == std::stoul(additionalFunctionParameters[0]);
                        }) != currentIntersection_p.end();
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
