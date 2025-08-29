#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/in_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool InIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters,
                                                bool setBased) {
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::find_if(lanelets.begin(), lanelets.end(),
                        [world, additionalFunctionParameters](const std::shared_ptr<Lanelet> &la) {
                            return la->hasLaneletType(LaneletType::intersection) and
                                   world->getRoadNetwork()
                                       ->getIntersectionByID(std::stoul(additionalFunctionParameters[0]))
                                       ->isMemberLanelet(la->getId());
                        }) != lanelets.end();
}

double InIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    throw std::runtime_error("InIntersectionPredicate does not support robust evaluation!");
}

Constraint InIntersectionPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    throw std::runtime_error("InIntersectionPredicate does not support constraint evaluation!");
}
InIntersectionPredicate::InIntersectionPredicate() : CommonRoadPredicate(false) {}
