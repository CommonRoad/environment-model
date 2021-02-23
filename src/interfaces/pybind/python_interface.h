//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_PYTHON_INTERFACE_H
#define ENV_MODEL_PYTHON_INTERFACE_H

#include <pybind11/pybind11.h>

namespace py = pybind11;

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles,
                            const py::list &py_trafficSigns, const py::list &py_trafficLights, const py::list &py_trafficIntersections,
                            const py::list &py_planningProblem);

uint8_t py_registerScenario(const py::int_ &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles,
                            const py::list &py_trafficSigns, const py::list &py_trafficLights, const py::list &py_trafficIntersections,);

uint8_t py_removeScenario(const py::int_ &ScenarioID);

PYBIND11_MODULE(env_model, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("registerScenario",
          py::overload_cast<const py::int_ &, const py::handle &, const py::list &, const py::list &, const py::list &, const py::list &, const py::list &>(
                  &py_registerScenario),
          "Add new scenario to C++ environment model", py::arg("ScenarioID"), py::arg("py_lanelets"),
          py::arg("py_obstacles"), py::arg("py_trafficSigns"), py::arg("py_trafficLights"),
          py::arg("py_trafficIntersections"), py::arg("py_planningProblem"));
    m.def("registerScenario",
          py::overload_cast<const py::int_ &, const py::handle &, const py::list &, const py::list &, const py::list &, const py::list &>(
                  &py_registerScenario),
          "Add new scenario without planning problem to C++ environment model", py::arg("ScenarioID"),
          py::arg("py_lanelets"),
          py::arg("py_obstacles"), py::arg("py_trafficSigns"), py::arg("py_trafficLights"),
          py::arg("py_trafficIntersections"));
    m.def("removeScenario", &py_removeScenario, "Remove a scenario from the environment model");
}

#endif //ENV_MODEL_PYTHON_INTERFACE_H
