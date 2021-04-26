//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_PYTHON_INTERFACE_H
#define ENV_MODEL_PYTHON_INTERFACE_H

#include <pybind11/pybind11.h>

namespace py = pybind11;

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles);

bool py_safe_distance_boolean_evaluation();

//uint8_t py_registerScenario2018b(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles);

//uint8_t py_removeScenario(const py::int_ &ScenarioID);

PYBIND11_MODULE(cpp_env_model, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("registerScenario", &py_registerScenario,
          "Add new scenario to C++ environment model", py::arg("ScenarioID"), py::arg("py_lanelets"),
          py::arg("py_obstacles"));
    m.def("safeDistanceBooleanEvaluation", &py_safe_distance_boolean_evaluation,
          "Boolean evaluation of safe distance predicate");
//    m.def("registerScenario2018b", &py_registerScenario2018b,
//          "Add 2018b scenario to C++ environment model", py::arg("ScenarioID"),
//          py::arg("py_lanelets"),py::arg("py_obstacles"));
//    m.def("removeScenario", &py_removeScenario, "Remove a scenario from the environment model");
}

#endif //ENV_MODEL_PYTHON_INTERFACE_H
