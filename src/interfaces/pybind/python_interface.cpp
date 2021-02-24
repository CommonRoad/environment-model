//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "translate_python_types.h"

uint8_t py_registerScenario(py::int_ &ScenarioID, py::handle &py_laneletNetwork, py::list &py_obstacles) {

    auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
    //auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
    //auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork);
    //auto tempObstacleContainer = TranslatePythonTypes::convertObstacle(py_laneletNetwork);

    // Todo give static obstacles through interface


    //------------------------------- Add scenario to Spot  -------------------------------
    // Create Scenario --> Move containers to scenario instance
//    Spot *SpotInterface = Spot::getInstance();
//    return SpotInterface->registerScenario(py::cast<int>(ScenarioID), std::move(tempLaneletContainer),
//                                           std::move(tempObstacleContainer), std::move(tempEgo),
//                                           std::move(tmpFieldOfView));
}

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles) {

}