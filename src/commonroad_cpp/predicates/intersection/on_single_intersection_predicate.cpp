#include <commonroad_cpp/predicates/intersection/on_single_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <stdexcept>

bool OnSingleIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    return intersection_operations::currentIntersection(timeStep, world, obstacleK).size() == 1;
}

double OnSingleIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("OnSingleIntersectionPredicate does not support robust evaluation!");
}

Constraint OnSingleIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnSingleIntersectionPredicate does not support constraint evaluation!");
}

OnSingleIntersectionPredicate::OnSingleIntersectionPredicate() : CommonRoadPredicate(true) {}
