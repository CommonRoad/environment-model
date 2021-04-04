// ######################################################################################################################
/// \file  TranslatePythonTypes.h
/// \brief Outsource the conversion from Commonroad specific variables from Python to their SPOT counterparts
///
/// These functions are solely called from PythonInterface.cpp
///
/// \author Florian Schmucker
/// \date 17.07.2019
#pragma once


#include "../../auxiliaryDefs/structs.h"
#include "../../src/world/Spot.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <variant>

namespace py = pybind11;

namespace TranslatePythonTypes {

// Fill the C++ vector with translated commonroad lanelets
std::vector<std::shared_ptr<vehicularLanelet>> convertLanelets(const py::handle &py_lanelets);

// Set the pointers from lanelets (Commonroad has them only as integer values with IDs)
void setLaneletPointer(const py::list &py_lanelets, std::vector<std::shared_ptr<vehicularLanelet>> &LaneletContainer);

// Fill the C++ vector with translated commonroad obstacles
uint8_t convertObstacles(const py::list &py_dynamicObstacles,
                         std::vector<std::shared_ptr<obstacle>> &ObstacleContainer);

// Create an egoVehicle from python information
void convertEgoVehicle(const py::list &py_planningProblem, std::shared_ptr<EgoVehicle> tempEgo);

}; // namespace TranslatePythonTypes
