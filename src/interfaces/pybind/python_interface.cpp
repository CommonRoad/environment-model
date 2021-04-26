//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "translate_python_types.h"
#include "predicates/predicate_evaluation.h"

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles, const py::list &py_egoVehicles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork, tempTrafficSignContainer,
                                                                      tempTrafficLightContainer);
    auto tempIntersectionContainer = TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
    auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles);

    PredicateEvaluation *eval = PredicateEvaluation::getInstance();

    return eval->registerScenario(py::cast<int>(ScenarioID),
            std::make_shared<RoadNetwork>(tempLaneletContainer, tempIntersectionContainer, tempTrafficSignContainer,
                        tempTrafficLightContainer), tempObstacleContainer, tempObstacleContainer);

    return 0;
}

