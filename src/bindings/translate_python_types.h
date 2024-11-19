#pragma once

#include "commonroad_cpp/world.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace TranslatePythonTypes {
/**
 * Converts Python lanelet objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return List of pointers to lanelet objects.
 */
std::vector<std::shared_ptr<Lanelet>> convertLanelets(const nb::handle &py_laneletNetwork,
                                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

/**
 * Converts Python dynamic obstacle objects to C++ representation.
 *
 * @param py_obstacles List of Python version of dynamic obstacles.
 * @return List of pointers to obstacle objects.
 */
std::vector<std::shared_ptr<Obstacle>> convertObstacles(const nb::list &py_obstacles);

/**
 * Converts Python State.
 *
 * @param py_state Python state object.
 * @return C++ State.
 */
std::shared_ptr<State> extractState(nb::handle py_state);

/**
 * Create dynamic obstacle.
 * @param py_singleObstacle Python dynamic obstacle.
 * @return C++ dynamic obstacle.
 */
std::shared_ptr<Obstacle> createDynamicObstacle(nb::handle py_singleObstacle);

/**
 * Create static obstacle.
 * @param py_singleObstacle Python static obstacle.
 * @return C++ static obstacle.
 */
std::shared_ptr<Obstacle> createStaticObstacle(nb::handle py_singleObstacle);

/**
 * Converts Python traffic sign objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic sign objects.
 */
std::vector<std::shared_ptr<TrafficSign>> convertTrafficSigns(const nb::handle &py_laneletNetwork);

/**
 * Converts Python traffic light objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic light objects.
 */
std::vector<std::shared_ptr<TrafficLight>> convertTrafficLights(const nb::handle &py_laneletNetwork);

/**
 * Converts Python intersection objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param lanelets List of pointers to lanelets.
 * @return List of pointers to intersection objects.
 */
std::vector<std::shared_ptr<Intersection>> convertIntersections(const nb::handle &py_laneletNetwork,
                                                                const std::vector<std::shared_ptr<Lanelet>> &lanelets);

/**
 * Converts Python stop line object to C++ representation.
 *
 * @param py_stopLine Python stop line object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return
 */
std::shared_ptr<StopLine> convertStopLine(const nb::handle &py_stopLine,
                                          const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                          const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

} // namespace TranslatePythonTypes
