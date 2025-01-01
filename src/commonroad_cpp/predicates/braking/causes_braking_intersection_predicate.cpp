#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h>

bool CausesBrakingIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {

    // check whether kth obstacle is in the projection domain of the reference of the pth vehicle -> otherwise ccs fails
    if (!obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)
             ->getCurvilinearCoordinateSystem()
             ->cartesianPointInProjectionDomain(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                obstacleK->getStateByTimeStep(timeStep)->getYPosition()))
        return false;
    auto distance{obstacleK->rearS(timeStep, obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)
                                                 ->getCurvilinearCoordinateSystem()) -
                  obstacleP->frontS(world->getRoadNetwork(), timeStep)};
    return parameters.getParam("dCauseBrakingIntersection") <= distance and
           distance <= parameters.getParam("dBrakingIntersection") and
           obstacleP->getStateByTimeStep(timeStep)->getAcceleration() < parameters.getParam("aBrakingIntersection");
}

Constraint CausesBrakingIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

double CausesBrakingIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

CausesBrakingIntersectionPredicate::CausesBrakingIntersectionPredicate() : CommonRoadPredicate(true) {}
