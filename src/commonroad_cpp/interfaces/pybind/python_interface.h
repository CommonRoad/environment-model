//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_PYTHON_INTERFACE_H
#define ENV_MODEL_PYTHON_INTERFACE_H

#include "commonroad_cpp/predicates/predicate_evaluation.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

uint8_t py_registerScenario(size_t scenarioId, size_t timeStep, const py::handle &py_laneletNetwork,
                            const py::list &py_obstacles, const py::list &py_egoVehicles);

bool py_safe_distance_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId,
                                         const py::list &py_obstaclesIds);

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact);

double py_safe_distance_robust_evaluation(size_t scenarioId, size_t timeStep,
                                          size_t py_egoVehicleId, const py::list &py_obstaclesIds);

double py_safe_distance_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                          double velocityP, double minAccelerationK,
                                                          double minAccelerationP, double tReact);

double py_safe_distance(double velocityK, double velocityP, double minAccelerationK,
                        double minAccelerationP, double tReact);

std::vector<int> createVectorFromPyList(const py::list &list);


// uint8_t py_removeScenario(const int &ScenarioID);

PYBIND11_MODULE(cpp_env_model, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("register_scenario", &py_registerScenario, "Add new scenario to C++ environment model", py::arg("scenarioId"),
          py::arg("timeStep"), py::arg("py_lanelets"), py::arg("py_obstacles"), py::arg("py_egoVehicles"));
    m.def("safe_distance_boolean_evaluation", &py_safe_distance_boolean_evaluation,
          "Boolean evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleIds"), py::arg("py_obstaclesIds"));
    m.def("safe_distance_boolean_evaluation", &py_safe_distance_boolean_evaluation_with_parameters,
          "Boolean evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"));
    m.def("safe_distance_robust_evaluation", &py_safe_distance_robust_evaluation,
          "Robust evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleIds"), py::arg("py_obstaclesIds"));
    m.def("safe_distance_robust_evaluation", &py_safe_distance_robust_evaluation_with_parameters,
          "Robust evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"));
  m.def("safe_distance", &py_safe_distance,
        "Calculation of safe distance", py::arg("velocityK"),
        py::arg("velocityP"), py::arg("minAccelerationK"),
        py::arg("minAccelerationP"), py::arg("tReact"));
}

#endif // ENV_MODEL_PYTHON_INTERFACE_H
