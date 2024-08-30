#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>

#include <commonroad_cpp/predicates/intersection/at_four_way_stop_predicate.h>

bool AtFourWayStopPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters) {
    // get current Intersection
    // look at all incomings of this intersection then look at all lanelets from each incoming
    // if lanelet from each incoming refereces a STOP sign, it is 4 way stop
    auto currentIntersection = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    if (currentIntersection == nullptr || currentIntersection->getIncomingGroups().size() != 4) {
        return false;
    }
    for (const auto &incoming : currentIntersection->getIncomingGroups())
        if (!std::any_of(
                incoming->getIncomingLanelets().begin(), incoming->getIncomingLanelets().end(),
                [](const std::shared_ptr<Lanelet> &la) { return la->hasTrafficSign(TrafficSignTypes::STOP_4_WAY); }))
            return false;
    return true;
}
double AtFourWayStopPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtFourWayStopPredicate does not support robust evaluation!");
}
Constraint AtFourWayStopPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtFourWayStopPredicate does not support constraint evaluation!");
}
AtFourWayStopPredicate::AtFourWayStopPredicate() : CommonRoadPredicate(false) {}
