//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
#include "commonroad_cpp/predicates/general/cut_in_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "translate_python_types.h"

void py_registerScenario(size_t scenarioId, size_t timeStep, const std::string &country,
                         const py::handle &py_laneletNetwork, const py::list &py_obstacles,
                         const py::list &py_egoVehicles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    auto tempLaneletContainer =
        TranslatePythonTypes::convertLanelets(py_laneletNetwork, tempTrafficSignContainer, tempTrafficLightContainer);
    auto tempIntersectionContainer =
        TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
    auto convertedCountry{RoadNetwork::matchStringToCountry(country)};
    auto roadNetwork = std::make_shared<RoadNetwork>(tempLaneletContainer, convertedCountry, tempIntersectionContainer,
                                                     tempTrafficSignContainer, tempTrafficLightContainer);
    auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles);
    for (const auto &obs : tempObstacleContainer) {
        for (unsigned long i = 0; i < obs->getTrajectoryLength(); ++i) {
            obs->setOwnLane(roadNetwork->getLanes(), i);
            obs->setReferenceLane(obs->getOwnLane());
        }
    }
    auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles);
    for (const auto &obs : tempEgoVehicleContainer) {
        for (unsigned long i = 0; i < obs->getTrajectoryLength(); ++i) {
            obs->setOwnLane(roadNetwork->getLanes(), i);
            obs->setReferenceLane(obs->getOwnLane());
        }
    }

    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();

    eval->registerScenario(scenarioId, timeStep, roadNetwork, tempObstacleContainer, tempEgoVehicleContainer);
}

void py_removeScenario(size_t scenarioId) {
    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();
    eval->removeScenario(scenarioId);
}

bool py_safe_distance_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                         size_t py_obstacleId) {
    SafeDistancePredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacle(py_obstacleId));
}

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact, double lengthK,
                                                         double lengthP) {
    return SafeDistancePredicate::booleanEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK,
                                                    minAccelerationP, tReact, lengthK, lengthP);
}

double py_safe_distance_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                          size_t py_obstacleId) {
    SafeDistancePredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                 world->findObstacle(py_obstacleId));
}

double py_safe_distance_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                          double velocityP, double minAccelerationK,
                                                          double minAccelerationP, double tReact, double lengthK,
                                                          double lengthP) {
    return SafeDistancePredicate::robustEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK,
                                                   minAccelerationP, tReact, lengthK, lengthP);
}

double py_safe_distance(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                        double tReact) {
    SafeDistancePredicate pred;
    return SafeDistancePredicate::computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact);
}

bool py_in_front_of_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                       size_t py_obstacleId) {
    InFrontOfPredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacle(py_obstacleId));
}

bool py_in_front_of_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP) {
    return InFrontOfPredicate::booleanEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}

double py_in_front_of_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                        size_t py_obstacleId) {
    InFrontOfPredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                 world->findObstacle(py_obstacleId));
}

double py_in_front_of_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK,
                                                        double lengthP) {
    return InFrontOfPredicate::robustEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}

bool py_in_same_lane_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                        size_t py_obstacleId) {
    InSameLanePredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacle(py_obstacleId));
}

bool py_unnecessary_braking_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                               size_t py_obstacleId) {
    UnnecessaryBrakingPredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}

double py_unnecessary_braking_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                                size_t py_obstacleId) {
    UnnecessaryBrakingPredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}

bool py_cut_in_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId) {
    CutInPredicate pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacle(py_obstacleId));
}