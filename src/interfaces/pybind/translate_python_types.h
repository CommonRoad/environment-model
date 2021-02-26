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
    /**
     * Converts Python lanelet objects to C++ representation.
     *
     * @param py_laneletNetwork Python lanelet network object.
     * @param trafficSigns List of pointers to converted traffic signs.
     * @param trafficLights List of pointers to converted traffic lights
     * @return List of pointers to lanelet objects.
     */
    std::vector<std::shared_ptr<Lanelet>> convertLanelets(const py::handle &py_laneletNetwork,
                                                          std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                                          std::vector<std::shared_ptr<TrafficLight>> trafficLights);

    /**
     * Converts Python obstacle objects to C++ representation.
     *
     * @param py_obstacles List of Python version of obstacles.
     * @return List of pointers to obstacle objects.
     */
    std::vector<std::shared_ptr<Obstacle>> convertObstacles(const py::handle &py_obstacles);

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
     * @return List of pointers to intersection objects.
     */
    std::vector<std::shared_ptr<Intersection>> convertIntersections(const py::handle &py_laneletNetwork);
}


#endif //ENV_MODEL_TRANSLATE_PYTHON_TYPES_H
