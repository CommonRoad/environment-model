//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "python_interface.h"
#include "translate_python_types.h"

uint8_t py_registerScenario(py::int_ &ScenarioID, py::handle &py_laneletNetwork, py::list &py_obstacles,
                            py::list &py_trafficSigns, py::list &py_trafficLights, py::list &py_trafficIntersections,
                            py::list &py_planningProblem) {
    auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork); // outsourced function

    //------------------------------- Lane creation  -------------------------------
    // Lanes are now created when constructing the scenario from transfered lanelets (by private member function)

    //------------------------------- Dynamic obstacles  -------------------------------
    // temporary pointer which stores translated dynamic obstacles
//    std::vector<std::shared_ptr<Obstacle>> tempObstacleContainer{};
//    uint8_t ObstacleConversionFlag = TranslatePythonTypes::convertObstacles(py_dynamicObstacles, tempObstacleContainer);
//    if (ObstacleConversionFlag) {
//        // Todo One undefined obstacle parameter brings the program to a stop --> Check if this is desired
//        // Otherwise only give an error and proceed with next obstacle (or set default values)
//        std::cout << "Error during obstacleConversion" << std::endl;
//        return ObstacleConversionFlag; // error during conversion of obstacles
//    }

    //------------------------------- Static obstacles  -------------------------------
    // Would be a new parameter given to function
    // Creates only staticObstacles and pushes them to tempObstacleContainer (obstacle = parentClass)
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