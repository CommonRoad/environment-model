//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_PYTHON_INTERFACE_H
#define ENV_MODEL_PYTHON_INTERFACE_H

#include <pybind11/pybind11.h>
#include "predicates/predicate_evaluation.h"

namespace py = pybind11;

uint8_t py_registerScenario(const size_t scenarioId, const size_t timeStep, const py::handle &py_laneletNetwork,
                            const py::list &py_obstacles, const py::list &py_egoVehicles);

bool py_safe_distance_boolean_evaluation(const size_t scenarioId, const size_t timeStep,
                                         const size_t py_egoVehicleId, const py::list &py_obstaclesIds);

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP,
                                                         double velocityK, double velocityP,
                                                         double minAccelerationK,
                                                         double minAccelerationP,
                                                         double tReact);

std::vector<int> createVectorFromPyList(const py::list& list);

//uint8_t py_registerScenario2018b(const int &ScenarioID, const py::handle &py_laneletNetwork, const py::list &py_obstacles);

//uint8_t py_removeScenario(const int &ScenarioID);

PYBIND11_MODULE(cpp_env_model, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("registerScenario", &py_registerScenario,
          "Add new scenario to C++ environment model", py::arg("scenarioId"),
          py::arg("timeStep"), py::arg("py_lanelets"),
          py::arg("py_obstacles"), py::arg("py_egoVehicles"));
    m.def("safeDistanceBooleanEvaluation", &py_safe_distance_boolean_evaluation,
          "Boolean evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
            py::arg("py_egoVehicleIds"), py::arg("py_obstaclesIds"));
    m.def("safeDistanceBooleanEvaluation", &py_safe_distance_boolean_evaluation_with_parameters,
         "Boolean evaluation of safe distance predicate using parameters directly",
         py::arg("lonPosK"), py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"),
         py::arg("minAccelerationK"), py::arg("minAccelerationP"), py::arg("tReact"));
//    m.def("registerScenario2018b", &py_registerScenario2018b,
//          "Add 2018b scenario to C++ environment model", py::arg("ScenarioID"),
//          py::arg("py_lanelets"),py::arg("py_obstacles"));
//    m.def("removeScenario", &py_removeScenario, "Remove a scenario from the environment model");
}

#endif //ENV_MODEL_PYTHON_INTERFACE_H
