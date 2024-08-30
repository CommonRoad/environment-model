#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/approach_intersection_predicate.h>
#include <commonroad_cpp/predicates/intersection/at_uncontrolled_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

bool AtUncontrolledIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {

    auto currentIntersection = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    if (currentIntersection == nullptr)
        return false;
    for (auto &incoming : currentIntersection->getIncomingGroups()) {
        for (auto &lanelet : incoming->getIncomingLanelets()) {
            if (!lanelet->getTrafficLights().empty() || !lanelet->getTrafficSigns().empty() ||
                !(lanelet->getStopLine() == nullptr)) {
                return false;
            }
        }
    }
    return true;
}
double AtUncontrolledIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtUncontrolledIntersectionPredicate does not support robust evaluation!");
}
Constraint AtUncontrolledIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtUncontrolledIntersectionPredicate does not support constraint evaluation!");
}
AtUncontrolledIntersectionPredicate::AtUncontrolledIntersectionPredicate() : CommonRoadPredicate(false) {}
