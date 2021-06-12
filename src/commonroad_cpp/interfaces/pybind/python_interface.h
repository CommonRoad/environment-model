//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_PYTHON_INTERFACE_H
#define ENV_MODEL_PYTHON_INTERFACE_H

#include "commonroad_cpp/commonroad_container.h"
#include <pybind11/pybind11.h>

#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"

namespace py = pybind11;

void py_registerScenario(size_t scenarioId, size_t timeStep, const std::string &country,
                         const py::handle &py_laneletNetwork, const py::list &py_obstacles,
                         const py::list &py_egoVehicles);

void py_removeScenario(size_t scenarioId);

template <typename T>
bool py_boolean_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId = 123456789);

template <typename T>
double py_robust_evaluation(size_t scenarioId, size_t timeStep, size_t py_egoVehicleId, size_t py_obstacleId = 123456789);

bool py_safe_distance_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                         double velocityP, double minAccelerationK,
                                                         double minAccelerationP, double tReact, double lengthK,
                                                         double lengthP);

double py_safe_distance_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double velocityK,
                                                          double velocityP, double minAccelerationK,
                                                          double minAccelerationP, double tReact, double lengthK,
                                                          double lengthP);

double py_safe_distance(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                        double tReact);

bool py_in_front_of_boolean_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP);

double py_in_front_of_robust_evaluation_with_parameters(double lonPosK, double lonPosP, double lengthK, double lengthP);


PYBIND11_MODULE(cpp_env_model, m) {
    m.doc() = "CommonRoad Python/C++ Interface";
    m.def("register_scenario", &py_registerScenario, "Add new scenario to C++ environment model", py::arg("scenarioId"),
          py::arg("timeStep"), py::arg("country"), py::arg("py_lanelets"), py::arg("py_obstacles"),
          py::arg("py_egoVehicles"));

    m.def("remove_scenario", &py_removeScenario, "Remove scenario to C++ environment model", py::arg("scenarioId"));

    m.def("safe_distance_boolean_evaluation", &py_boolean_evaluation<SafeDistancePredicate>,
          "Boolean evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("safe_distance_boolean_evaluation", &py_safe_distance_boolean_evaluation_with_parameters,
          "Boolean evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("safe_distance_robust_evaluation", &py_robust_evaluation<SafeDistancePredicate>,
          "Robust evaluation of safe distance predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("safe_distance_robust_evaluation", &py_safe_distance_robust_evaluation_with_parameters,
          "Robust evaluation of safe distance predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("velocityK"), py::arg("velocityP"), py::arg("minAccelerationK"),
          py::arg("minAccelerationP"), py::arg("tReact"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("safe_distance", &py_safe_distance, "Calculation of safe distance", py::arg("velocityK"),
          py::arg("velocityP"), py::arg("minAccelerationK"), py::arg("minAccelerationP"), py::arg("tReact"));

    m.def("in_front_of_boolean_evaluation", &py_boolean_evaluation<InFrontOfPredicate>,
          "Boolean evaluation of in front of predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_front_of_boolean_evaluation", &py_in_front_of_boolean_evaluation_with_parameters,
          "Boolean evaluation of in front of predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("in_front_of_robust_evaluation", &py_robust_evaluation<InFrontOfPredicate>,
          "Robust evaluation of in front of predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("in_front_of_robust_evaluation", &py_in_front_of_robust_evaluation_with_parameters,
          "Robust evaluation of in front of predicate using parameters directly", py::arg("lonPosK"),
          py::arg("lonPosP"), py::arg("lengthK"), py::arg("lengthP"));

    m.def("in_same_lane_boolean_evaluation", &py_boolean_evaluation<InSameLanePredicate>,
          "Boolean evaluation of in same lane predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId"));

    m.def("unnecessary_braking_boolean_evaluation", &py_boolean_evaluation<UnnecessaryBrakingPredicate>,
          "Boolean evaluation of unnecessary braking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 123456789);

    m.def("unnecessary_braking_robust_evaluation", &py_robust_evaluation<UnnecessaryBrakingPredicate>,
          "Robust evaluation of unnecessary braking predicate", py::arg("scenarioId"), py::arg("time_step"),
          py::arg("py_egoVehicleId"), py::arg("py_obstacleId") = 123456789);
}

#endif // ENV_MODEL_PYTHON_INTERFACE_H
