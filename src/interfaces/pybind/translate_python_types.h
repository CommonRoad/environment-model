//
// Created by Sebastian Maierhofer on 23.02.21.
//

#ifndef ENV_MODEL_TRANSLATE_PYTHON_TYPES_H
#define ENV_MODEL_TRANSLATE_PYTHON_TYPES_H

#include "../../roadNetwork/lanelet/lanelet.h"
#include "../../obstacle/obstacle.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace TranslatePythonTypes {

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief Convert the vehicular lanelets from Commonroad to the C++ counterpart
///
///     This function is called from the PythonInterface.cpp
///
/// \param[in] py_lanelets = Commonroad version of lanelets
/// \param[in] laneletContainer = Container for SPOT version of lanelets (is filled here and should be empty before)
/// \note The functions inside the namesapce "TranslatePythonTypes" are only used by the python-interface
    // Fill the C++ vector with translated commonroad lanelets
    std::vector<std::shared_ptr<Lanelet>> convertLanelets(const py::handle &py_lanelets);

// Set the pointers from lanelets (Commonroad has them only as integer values with IDs)
 //   void setLaneletPointer(const py::list &py_lanelets, std::vector<std::shared_ptr<Lanelet>> &LaneletContainer);

// Fill the C++ vector with translated commonroad obstacles
 //   uint8_t convertObstacles(const py::list &py_dynamicObstacles,
 //                            std::vector<std::shared_ptr<Obstacle>> &ObstacleContainer);
}


#endif //ENV_MODEL_TRANSLATE_PYTHON_TYPES_H
