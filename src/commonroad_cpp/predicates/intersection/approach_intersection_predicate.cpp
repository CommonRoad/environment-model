#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool ApproachIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    bool approachIntersection = false;
    for (const auto &lanelet : lanelets) {
        if (!world->getRoadNetwork()
                 ->getIntersectionByID(std::stoul(additionalFunctionParameters[0]))
                 ->isMemberLanelet(lanelet->getId()))
            continue;
        if (lanelet->hasLaneletType(LaneletType::intersection))
            return false;
        if (lanelet->hasLaneletType(LaneletType::incoming))
            approachIntersection = true;
    }
    return approachIntersection;
}

double ApproachIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support robust evaluation!");
}

Constraint ApproachIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support constraint evaluation!");
}
ApproachIntersectionPredicate::ApproachIntersectionPredicate() : CommonRoadPredicate(false) {}
