//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "python_interface.h"
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
    auto roadNetwork = std::make_shared<RoadNetwork>(tempLaneletContainer, convertedCountry, tempTrafficSignContainer,
                                                     tempTrafficLightContainer, tempIntersectionContainer);
    auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles);
    auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles);

    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();

    eval->registerScenario(scenarioId, timeStep, roadNetwork, tempObstacleContainer, tempEgoVehicleContainer);
}

void py_removeScenario(size_t scenarioId) {
    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();
    eval->removeScenario(scenarioId);
}

template <typename T>
bool py_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId) {
    T pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacle(py_obstacleId));
}

template <typename T>
bool py_boolean_single_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId) {
    T pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}

template <typename T>
double py_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId) {
    T pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                 world->findObstacle(py_obstacleId));
}

template <typename T>
double py_robust_single_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId) {
    T pred;
    std::shared_ptr<CommonRoadContainer> CommonRoadContainer = CommonRoadContainer::getInstance();
    auto world = CommonRoadContainer->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact, double lengthK,
                                                         double lengthP) {
    return SafeDistancePredicate::booleanEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK,
                                                    minAccelerationP, tReact, lengthK, lengthP);
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

bool py_in_front_of_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP) {
    return InFrontOfPredicate::booleanEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}

double py_in_front_of_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK,
                                                        double lengthP) {
    return InFrontOfPredicate::robustEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}
