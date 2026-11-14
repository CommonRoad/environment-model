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
    // Cache both parsed values; params are fixed per scenario instance. Saves some resource compared to using
    // std::stoul at every function call
    if (additionalFunctionParameters[1] != cachedIdStr_) {
        cachedIdStr_ = additionalFunctionParameters[1];
        cachedId_ = std::stoul(additionalFunctionParameters[1]);
    }
    if (additionalFunctionParameters[0] != cachedTypeStr_) {
        cachedTypeStr_ = additionalFunctionParameters[0];
        cachedType_ = intersection_operations::matchStringToIntersectionType(additionalFunctionParameters[0]);
    }
    const auto intersectionId = cachedId_;
    const auto intersectionType = cachedType_.value();
    return world->getRoadNetwork()->getIntersectionByID(intersectionId)->hasIntersectionType(intersectionType);
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
