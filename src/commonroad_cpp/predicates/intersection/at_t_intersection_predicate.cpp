#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/at_t_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>

bool AtTIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters) {
    const auto currentIntersection = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    if (currentIntersection == nullptr || currentIntersection->getIncomingGroups().size() != 3) {
        return false;
    }
    const auto incoming_1 = currentIntersection->getIncomingGroups().at(0)->getIncomingLanelets().at(0);
    const auto incoming_2 = currentIntersection->getIncomingGroups().at(1)->getIncomingLanelets().at(0);
    const auto incoming_3 = currentIntersection->getIncomingGroups().at(2)->getIncomingLanelets().at(0);

    // get the orientation from each incoming lanelet with the vertices
    const auto orientation_incoming_1 = geometric_operations::getOrientationInDeg(incoming_1);
    const auto orientation_incoming_2 = geometric_operations::getOrientationInDeg(incoming_2);
    const auto orientation_incoming_3 = geometric_operations::getOrientationInDeg(incoming_3);

    // now get through the three cases how the incomings can be located
    // first case: T is incoming_1
    if (geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_2) &&
        geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_3)) {
        return geometric_operations::is180Deg(orientation_incoming_2, orientation_incoming_3);
    }
    // second case: T is incoming_2
    if (geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_1) &&
        geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_3)) {
        return geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_3);
    }
    // third case: T is incoming_3
    if (geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_1) &&
        geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_2)) {
        return geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_2);
    }
    return false;
}
double AtTIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtTIntersectionPredicate does not support robust evaluation!");
}
Constraint AtTIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtTIntersectionPredicate does not support constraint evaluation!");
}
AtTIntersectionPredicate::AtTIntersectionPredicate() : CommonRoadPredicate(false) {}
