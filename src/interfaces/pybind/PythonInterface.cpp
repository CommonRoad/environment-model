//######################################################################################################################
// DESCRIPTION: Implementation of the interface functions defined in "PythonInterface.h"
//######################################################################################################################

#include "PythonInterface.h"
#include "../../src/world/Journal.h"
#include "TranslatePythonTypes.h"
#include <cmath>

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief This function is called from the Python-interface to create a new scenario
///
///     Create a new scenarios (multiple ones can be hold by SPOT at the same time
///     A new instance of the class defined in \ref Scenario.h is build.
///     Needs information for Lanelets, Obstacles and egoVehicle
///
///     Exemplary usage:
///  \code{.py}
///     scenario, planning_problem_set = CommonRoadFileReader(scenario_path + scenario_name + '.xml').open()
///     planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
///     lanelets = scenario.lanelet_network.lanelets
///     dynamicObstacles = scenario.dynamic_obstacles
///     planning_list = [planning_problem]
///
///     result = registerScenario(1, lanelets, dynamicObstacles, planning_list)
/// \endcode
///
/// \param[in] ScenarioID Unique Id for differentiation between different Scenarios
/// \param[in] py_lanelets Python-List of instances of the Commonroad class "Lanelet"
/// \param[in] py_dynamicObstacles Python-List of instances of the Commonroad class "DynamicObstacle"
/// \param[in] py_planningProblem One instance of the Commonroad class "PlanningProblem" wrapped in a python-list
/// \param[in] py_fieldOfView <b>[optional argument]</b> polygon as np.array (n x 2) describing the field of view of the
/// ego vehicle
///
/// \returns Integer that indicates if function terminated successfully
/// \note Variables with python specific types contain a "py_" prefix
uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_dynamicObstacles,
                            const py::list &py_planningProblem) {
    // tempLaneletContainer will be filled with the converted lanelets from the python list
    auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(py_laneletNetwork); // outsourced function

    //------------------------------- Lane creation  -------------------------------
    // Lanes are now created when constructing the scenario from transfered lanelets (by private member function)

    //------------------------------- Dynamic obstacles  -------------------------------
    // temporary pointer which stores translated dynamic obstacles
    std::vector<std::shared_ptr<obstacle>> tempObstacleContainer{};
    uint8_t ObstacleConversionFlag = TranslatePythonTypes::convertObstacles(py_dynamicObstacles, tempObstacleContainer);
    if (ObstacleConversionFlag) {
        // Todo One undefined obstacle parameter brings the program to a stop --> Check if this is desired
        // Otherwise only give an error and proceed with next obstacle (or set default values)
        std::cout << "Error during obstacleConversion" << std::endl;
        return ObstacleConversionFlag; // error during conversion of obstacles
    }

    //------------------------------- Static obstacles  -------------------------------
    // Would be a new parameter given to function
    // Creates only staticObstacles and pushes them to tempObstacleContainer (obstacle = parentClass)
    // Todo give static obstacles through interface

    //------------------------------- Set ego vehicle -------------------------------
    // Define the egoVehicle (this is currently defined as special kind of obstacle)
    std::shared_ptr<EgoVehicle> tempEgo = std::make_shared<EgoVehicle>();
    TranslatePythonTypes::convertEgoVehicle(py_planningProblem, tempEgo); // outsourced function

    //------------------------------- Field of view -------------------------------
    std::shared_ptr<std::vector<vertice>> tmpFieldOfView;

    if (py_fieldOfView && py_fieldOfView.size() > 0) {
        // Copy np.ndarray based on https://stackoverflow.com/questions/49582252/pybind-numpy-access-2d-nd-arrays
        auto py_buffer = py_fieldOfView.request(); // request() accesses numpy header of type py::buffer_info
        if (py_buffer.shape[0] < 3 || py_buffer.shape[1] != 2) {
            throw std::invalid_argument(
                "Dimensions of py_fieldOfView are incompatible. Expected n x 2 with n >= 3, but received " +
                std::to_string(py_buffer.shape[0]) + " x " + std::to_string(py_buffer.shape[1]));
        }
        std::vector<vertice> VectorFieldOfView;
        double *py_verticeList = (double *)py_buffer.ptr;
        for (ssize_t idx = 0; idx < py_buffer.shape[0]; idx++) {
            VectorFieldOfView.emplace_back(vertice{py_verticeList[idx * 2], py_verticeList[idx * 2 + 1]});
        }
        tmpFieldOfView = std::make_shared<std::vector<vertice>>(VectorFieldOfView);
        std::cout << "Successfully converted field of view from Python" << std::endl;
    }

    //------------------------------- Add scenario to Spot  -------------------------------
    // Create Scenario --> Move containers to scenario instance
    Spot *SpotInterface = Spot::getInstance();
    return SpotInterface->registerScenario(py::cast<int>(ScenarioID), std::move(tempLaneletContainer),
                                           std::move(tempObstacleContainer), std::move(tempEgo),
                                           std::move(tmpFieldOfView));
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief This function is called from the Python-interface to create a new scenario
///
/// \overload
uint8_t py_registerScenario(py::int_ &ScenarioID, py::handle &py_laneletNetwork, py::list &py_dynamicObstacles,
                            py::list &py_planningProblem) {
    // create dummy and call overlaoded py_registerScenario (default argument did not work with bindings)
    auto dummy_py_fieldOfView = py::array_t<double>();
    return py_registerScenario(ScenarioID, py_laneletNetwork, py_dynamicObstacles, py_planningProblem, dummy_py_fieldOfView);
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief This function is called to remove the specified scenario
///
/// All occupied memory space is freed upon scenario deletion
///
/// \note In a later version it will be possible to copy scenarios: the containers of the scenarios only remove
/// their shared pointers (reducing index by one). Thus other scenarios will be able to hold the same information,
/// which is only deleted by the last holder
///
/// \param ScenarioID Has to be an existing ID
/// \return Indicator if removal was successful
uint8_t py_removeScenario(py::int_ &ScenarioID) {
    Spot *SpotInterface = Spot::getInstance();
    return SpotInterface->removeScenario(py::cast<int>(ScenarioID));
}

