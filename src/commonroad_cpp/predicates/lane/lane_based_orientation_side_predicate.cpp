#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/lane_based_orientation_side_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool LaneBasedOrientationSidePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    return (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) > 0) or
           (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::right and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) < 0);
}

double LaneBasedOrientationSidePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("LaneBasedOrientationSidePredicate does not support robust evaluation!");
}

Constraint LaneBasedOrientationSidePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("LaneBasedOrientationSidePredicate does not support constraint evaluation!");
}
LaneBasedOrientationSidePredicate::LaneBasedOrientationSidePredicate() : CommonRoadPredicate(true) {}
