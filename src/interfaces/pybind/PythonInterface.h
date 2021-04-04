// ######################################################################################################################
/// \file  PythonInterface.h
/// \brief Functions are exposed over pybind11 so that they can be called from Commonroad in Python.
///
///        Note that all information is copied to the defined C++-types and -classes.
///        This means that there are no changes to the variables that are defined in Python.
///
///        Implementation philosophy uses as few exposure as possible: Python gives list objects to the functions and
///        doesn't have to know about specific implemenations on the C++ side
///
///         Expected program flow in Python:
///         1. Add one or more scenario with unique IDs (scenario with lanelets and obstacles + egoVehicle)
///         2. Calculate Occupancies for specific scenario (for the given road network)
///         3. Update scenarios as often as desired (add lanelets, obstacles,...)
///         4. Delete scenarios from SPOT
///
/// \author Markus Koschi
/// \author Florian Schmucker
/// \date 28.05.2019
/// \note This interface is part of the SPOT extension for Commonroad.
///     See https://commonroad.in.tum.de/

#pragma once

#include "../../auxiliaryDefs/structs.h"
#include "../../src/world/Spot.h"
#include "pybind11/pytypes.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <variant>

namespace py = pybind11;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Doxygen documentation of interface
/// \defgroup group1 Python_Interface
/// @{
/// \brief Definition of the Python interface
///
/// Functions are exposed over pybind11 so that they can be called from Commonroad in Python.
/// More information can be found in \ref PythonInterface.h
/// - uint8_t py_registerScenario(py::int_& ScenarioID, py::list& py_lanelets, py::list& py_dynamicObstacles, py::list&
/// py_planningProblem, py::array_t<double> &py_fieldOfView);
/// - uint8_t py_registerScenario(py::int_& ScenarioID, py::list& py_lanelets, py::list& py_dynamicObstacles, py::list&
/// py_planningProblem);
/// - uint8_t py_updateScenario();
/// - uint8_t py_writeScenarioToXML(py::int_& ScenarioID, py::str& FileName);
/// - py::list py_doOccupancyPrediction(py::int_& ScenarioID, py::float_& StartTime, py::float_& TimeStep, py::float_&
/// EndTime, py::int_& NumThreads);
/// - uint8_t py_removeScenario(py::int_& ScenarioID);
/// @}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Declaration of interface functions
uint8_t py_setLoggingMode(py::int_ flag);
uint8_t py_registerScenario(py::int_ &ScenarioID, py::handle &py_laneletNetwork, py::list &py_dynamicObstacles,
                            py::list &py_planningProblem, py::array_t<double> &py_fieldOfView);
uint8_t py_registerScenario(py::int_ &ScenarioID, py::handle &py_laneletNetwork, py::list &py_dynamicObstacles,
                            py::list &py_planningProblem);
uint8_t py_updateScenario();
uint8_t py_writeScenarioToXML(py::int_ &ScenarioID, py::str &FileName);
py::list py_doOccupancyPrediction(py::int_ &ScenarioID, py::float_ &StartTime, py::float_ &TimeStep,
                                  py::float_ &EndTime, py::int_ &NumThreads);
uint8_t py_removeScenario(py::int_ &ScenarioID);
uint8_t py_updateProperties(
    py::int_ &ScenarioId,
    std::map<std::string, std::map<size_t, std::map<std::string, std::variant<bool, float>>>> UpdateParams);

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Define the interface-functions with pybind11:
// General structure
// spot = name of the project
// m =
// m.def("Name of function in Python",
//       "Adress of function in C++",
//       "Description", "Ownership")
PYBIND11_MODULE(spot, m) {
    // Definition of the externally available functions in python
    // These can be used in Python with e.g. "from spot import removeScenario"
    m.doc() = "SPOT to Python with pybind11";
    m.def("setLoggingMode", &py_setLoggingMode, "Sets the debug mode in Journal");
    m.def(
        "registerScenario",
        py::overload_cast<py::int_ &, py::handle &, py::list &, py::list &, py::array_t<double> &>(&py_registerScenario),
        "Add new scenario to SPOT", py::arg("ScenarioID"), py::arg("py_lanelets"), py::arg("py_dynamicObstacles"),
        py::arg("py_planningProblem"), py::arg("py_fieldOfView"));
    m.def("registerScenario", py::overload_cast<py::int_ &, py::handle &, py::list &, py::list &>(&py_registerScenario),
          "Add new scenario to SPOT without field of view", py::arg("ScenarioID"), py::arg("py_lanelets"),
          py::arg("py_dynamicObstacles"), py::arg("py_planningProblem"));
    m.def("updateScenario", &py_updateScenario, "Update an existing scenario");
    m.def("writeScenarioToXML", &py_writeScenarioToXML, "Export scenario to XML");
    m.def("removeScenario", &py_removeScenario, "Remove an existing scenario from SPOT");
    m.def("updateProperties", &py_updateProperties, "Update scenario properties");
    m.def("doOccupancyPrediction", &py_doOccupancyPrediction, "Do the occupancy prediction for a specific scenario",
          py::return_value_policy::take_ownership);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
