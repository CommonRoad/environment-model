#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/lane/in_single_lane_predicate.h>

bool InSingleLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters) {
    return obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep).size() == 1;
}

double InSingleLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InSingleLanePredicate does not support robust evaluation!");
}

Constraint InSingleLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InSingleLanePredicate does not support constraint evaluation!");
}
InSingleLanePredicate::InSingleLanePredicate() : CommonRoadPredicate(false) {}
