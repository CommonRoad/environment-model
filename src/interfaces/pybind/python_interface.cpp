//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "translate_python_types.h"
#include "predicates/safe_distance_predicate.h"

uint8_t py_registerScenario(const py::int_ &scenarioId, const py::int_ &timeStep, const py::handle &py_laneletNetwork,
                            const py::list &py_obstacles, const py::list &py_egoVehicles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork, tempTrafficSignContainer,
                                                                      tempTrafficLightContainer);
    auto tempIntersectionContainer = TranslatePythonTypes::convertIntersections(py_laneletNetwork,
                                                                                tempLaneletContainer);
    auto roadNetwork = std::make_shared<RoadNetwork>(tempLaneletContainer, tempIntersectionContainer,
                                                     tempTrafficSignContainer, tempTrafficLightContainer);
    auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles);
    for (const auto &obs : tempObstacleContainer) {
        for (int i = 0; i < obs->getTrajectoryLength(); ++i) {
            obs->setOwnLane(roadNetwork->getLanes(), i);
            obs->setReferenceLane(obs->getOwnLane());
        }
    }
    auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles);
    for (const auto &obs : tempEgoVehicleContainer) {
        for (int i = 0; i < obs->getTrajectoryLength(); ++i) {
            obs->setOwnLane(roadNetwork->getLanes(), i);
            obs->setReferenceLane(obs->getOwnLane());
        }
    }

    PredicateEvaluation *eval = PredicateEvaluation::getInstance();

    return eval->registerScenario(py::cast<int>(scenarioId), py::cast<int>(timeStep), roadNetwork,
                                  tempObstacleContainer, tempEgoVehicleContainer);
}

bool py_safe_distance_boolean_evaluation(const py::int_ &scenarioId,
                                         const py::int_& timeStep,
                                         const py::list &py_obstacleIds,
                                         const py::list &py_egoVehicleIds){
    SafeDistancePredicate pred;
    PredicateEvaluation *predicateEvaluation = PredicateEvaluation::getInstance();
    auto world = predicateEvaluation->findWorld(scenarioId);
    return pred.booleanEvaluation(timeStep, world,
                                  world->findObstacles(createVectorFromPyList(py_obstacleIds)).at(0),
                                  world->findObstacles(createVectorFromPyList(py_egoVehicleIds)).at(0));
}

std::vector<int> createVectorFromPyList(const py::list& list){
    std::vector<int> vec { };
    vec.reserve(list.size());
    for (const auto &elem : list) {
        vec.emplace_back(py::cast<int>(elem));
    }
    return vec;
}