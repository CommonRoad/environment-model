//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "translate_python_types.h"

uint8_t py_registerScenario(const size_t scenarioId, const size_t timeStep, const py::handle &py_laneletNetwork,
                            const py::list &py_obstacles, const py::list &py_egoVehicles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    auto tempLaneletContainer =
        TranslatePythonTypes::convertLanelets(py_laneletNetwork, tempTrafficSignContainer, tempTrafficLightContainer);
    auto tempIntersectionContainer =
        TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
    auto roadNetwork = std::make_shared<RoadNetwork>(tempLaneletContainer, tempIntersectionContainer,
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

    std::shared_ptr<PredicateEvaluation> eval = PredicateEvaluation::getInstance();

    return eval->registerScenario(scenarioId, timeStep, roadNetwork, tempObstacleContainer, tempEgoVehicleContainer);
}

uint8_t py_removeScenario(const size_t scenarioId) {
    std::shared_ptr<PredicateEvaluation> eval = PredicateEvaluation::getInstance();
    return eval->removeScenario(scenarioId);
}

std::vector<size_t> createVectorFromPyList(const py::list &list) {
    std::vector<size_t> vec{};
    vec.reserve(list.size());
    for (const auto &elem : list) {
        vec.emplace_back(py::cast<int>(elem));
    }
    return vec;
}

bool py_safe_distance_boolean_evaluation(const size_t scenarioId, const size_t timeStep, const size_t py_egoVehicleId,
                                         const py::list &py_obstacleIds) {
    std::cout << "print 1" << '\n';
    SafeDistancePredicate pred;
    std::cout << "print 2" << '\n';
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    std::cout << "print 3" << '\n';
    auto world = predicateEvaluation->findWorld(scenarioId);
    std::cout << "print 4" << '\n';
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0));
}

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact, double lengthK,
                                                         double lengthP) {
    return SafeDistancePredicate::booleanEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK,
                                                    minAccelerationP, tReact, lengthK, lengthP);
}

double py_safe_distance_robust_evaluation(const size_t scenarioId, const size_t timeStep, const size_t py_egoVehicleId,
                                          const py::list &py_obstacleIds) {
    SafeDistancePredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                 world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0));
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
                                       const py::list &py_obstacleIds) {
    InFrontOfPredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0));
}

bool py_in_front_of_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP) {
    return InFrontOfPredicate::booleanEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}

double py_in_front_of_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                        const py::list &py_obstacleIds) {
    InFrontOfPredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                 world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0));
}

double py_in_front_of_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK,
                                                        double lengthP) {
    return InFrontOfPredicate::robustEvaluation(lonPosK, lonPosP, lengthK, lengthP);
}

bool py_in_same_lane_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                        const py::list &py_obstacleIds) {
    InSameLanePredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId),
                                  world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0));
}

bool py_unnecessary_braking_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                               const py::list &py_obstacleIds) {
    UnnecessaryBrakingPredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}

double py_unnecessary_braking_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                                const py::list &py_obstacleIds) {
    UnnecessaryBrakingPredicate pred;
    std::shared_ptr<PredicateEvaluation> predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.robustEvaluation(timeStep, world, world->findObstacle(py_egoVehicleId));
}
