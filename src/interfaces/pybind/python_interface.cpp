//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "translate_python_types.h"

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork, tempTrafficSignContainer,
                                                                      tempTrafficLightContainer);
    auto tempIntersectionContainer = TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
    //auto tempObstacleContainer = TranslatePythonTypes::convertDynamicObstacles(py_obstacles);

    return 1;
}
