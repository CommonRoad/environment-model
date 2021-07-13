//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace TranslatePythonTypes {
/**
 * Converts Python lanelet objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return List of pointers to lanelet objects.
 */
std::vector<std::shared_ptr<Lanelet>> convertLanelets(const py::handle &py_laneletNetwork,
                                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

/**
 * Converts Python dynamic obstacle objects to C++ representation.
 *
 * @param py_obstacles List of Python version of dynamic obstacles.
 * @return List of pointers to obstacle objects.
 */
std::vector<std::shared_ptr<Obstacle>> convertObstacles(const py::list &py_obstacles);

/**
 * Converts Python traffic sign objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic sign objects.
 */
std::vector<std::shared_ptr<TrafficSign>> convertTrafficSigns(const py::handle &py_laneletNetwork);

/**
 * Converts Python traffic light objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic light objects.
 */
std::vector<std::shared_ptr<TrafficLight>> convertTrafficLights(const py::handle &py_laneletNetwork);

/**
 * Converts Python intersection objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param lanelets List of pointers to lanelets.
 * @return List of pointers to intersection objects.
 */
std::vector<std::shared_ptr<Intersection>> convertIntersections(const py::handle &py_laneletNetwork,
                                                                const std::vector<std::shared_ptr<Lanelet>> &lanelets);

/**
 * Converts Python stop line object to C++ representation.
 *
 * @param py_stopLine Python stop line object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return
 */
std::shared_ptr<StopLine> convertStopLine(const py::handle &py_stopLine,
                                          const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                          const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

} // namespace TranslatePythonTypes
