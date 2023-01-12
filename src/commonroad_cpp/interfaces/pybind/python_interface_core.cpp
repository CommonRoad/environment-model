//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>
#include <string>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/signal_state.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "python_interface_core.h"

#include <pybind11/stl.h> // for vector

namespace py = pybind11;

void init_python_interface_core(py::module_ &m) {
    py::enum_<ObstacleType>(m, "ObstacleType")
        .value("car", ObstacleType::car)
        .value("truck", ObstacleType::truck)
        .value("pedestrian", ObstacleType::pedestrian)
        .value("bus", ObstacleType::bus)
        .value("unknown", ObstacleType::unknown)
        .value("vehicle", ObstacleType::vehicle)
        .value("bicycle", ObstacleType::bicycle)
        .value("priority_vehicle", ObstacleType::priority_vehicle)
        .value("train", ObstacleType::train)
        .value("motorcycle", ObstacleType::motorcycle)
        .value("taxi", ObstacleType::taxi)
        .export_values();

    py::enum_<LineMarking>(m, "LineMarking")
        .value("solid", LineMarking::solid)
        .value("dashed", LineMarking::dashed)
        .value("broad_dashed", LineMarking::broad_dashed)
        .value("broad_solid", LineMarking::broad_solid)
        .value("unknown", LineMarking::unknown)
        .value("no_marking", LineMarking::no_marking)
        .export_values();

    py::class_<Shape>(m, "Shape");

    py::class_<State, std::shared_ptr<State>>(m, "State")
        .def_property("time_step", &State::getTimeStep, &State::setTimeStep)
        .def_property("x", &State::getXPosition, &State::setXPosition)
        .def_property("y", &State::getYPosition, &State::setYPosition)
        .def_property("lon", &State::getLonPosition, &State::setLonPosition)
        .def_property("lat", &State::getLatPosition, &State::setLatPosition)
        .def_property("global_orientation", &State::getGlobalOrientation, &State::setGlobalOrientation)
        .def_property("curvilinear_orientation", &State::getCurvilinearOrientation, &State::setCurvilinearOrientation)
        .def_property("velocity", &State::getVelocity, &State::setVelocity)
        .def_property("acceleration", &State::getAcceleration, &State::setAcceleration);

    py::class_<SignalState, std::shared_ptr<SignalState>>(m, "SignalState")
        .def_property("time_step", &SignalState::getTimeStep, &SignalState::setTimeStep)
        .def_property("horn", &SignalState::isHorn, &SignalState::setHorn)
        .def_property("indicator_left", &SignalState::isIndicatorLeft, &SignalState::setIndicatorLeft)
        .def_property("indicator_right", &SignalState::isIndicatorRight, &SignalState::setIndicatorRight)
        .def_property("brakingLights", &SignalState::isBrakingLights, &SignalState::setBrakingLights)
        .def_property("hazard_warning_lights", &SignalState::isHazardWarningLights,
                      &SignalState::setHazardWarningLights)
        .def_property("flashing_blue_lights", &SignalState::isFlashingBlueLights, &SignalState::setFlashingBlueLights);

    py::class_<ActuatorParameters>(m, "ActuatorParameters")
        .def_property_readonly("v_max", &ActuatorParameters::getVmax)
        .def_property_readonly("a_max", &ActuatorParameters::getAmax)
        .def_property_readonly("a_max_long", &ActuatorParameters::getAmaxLong)
        .def_property_readonly("a_min_long", &ActuatorParameters::getAminLong)
        .def_property_readonly("a_braking", &ActuatorParameters::getAbraking);

    py::class_<SensorParameters>(m, "SensorParameters")
        .def_property_readonly("fov_front", &SensorParameters::getFieldOfViewFront)
        .def_property_readonly("fov_rear", &SensorParameters::getFieldOfViewRear)
        .def_property_readonly("reaction_time", &SensorParameters::getReactionTime);

    py::class_<Obstacle, std::shared_ptr<Obstacle>>(m, "Obstacle")
        .def_property("id", &Obstacle::getId, &Obstacle::setId)
        .def_property("type", &Obstacle::getObstacleType, &Obstacle::setObstacleType)

        .def_property_readonly("current_state", &Obstacle::getCurrentState)
        .def_property_readonly("current_signal_state", &Obstacle::getCurrentSignalState)

        .def("shape", &Obstacle::getGeoShape)

        .def("occupied_lanes", &Obstacle::getOccupiedLanes)
        .def("driving_path_lanes", &Obstacle::getDrivingPathLanes)

        .def("time_step_exists", &Obstacle::timeStepExists)

        .def_property("actuator_parameters", &Obstacle::getActuatorParameters, &Obstacle::setActuatorParameters)
        .def_property("sensor_parameters", &Obstacle::getSensorParameters, &Obstacle::setSensorParameters)

        .def("get_state_by_time_step", &Obstacle::getStateByTimeStep)
        .def("reference_lane_by_time_step", &Obstacle::getReferenceLane)

        .def("get_time_steps", &Obstacle::getTimeSteps);

    py::class_<StopLine, std::shared_ptr<StopLine>>(m, "StopLine");

    py::class_<TrafficSign, std::shared_ptr<TrafficSign>>(m, "TrafficSign");

    py::class_<TrafficLight, std::shared_ptr<TrafficLight>>(m, "TrafficLight");

    py::class_<Lanelet, std::shared_ptr<Lanelet>>(m, "Lanelet")
        .def_property("id", &Lanelet::getId, &Lanelet::setId)
        .def_property("left_border_vertices", &Lanelet::getLeftBorderVertices, &Lanelet::setLeftBorderVertices)
        .def_property("right_border_vertices", &Lanelet::getRightBorderVertices, &Lanelet::setRightBorderVertices)
        .def_property("lanelet_types", &Lanelet::getLaneletTypes, &Lanelet::setLaneletTypes)
        .def_property("line_marking_left", &Lanelet::getLineMarkingLeft, &Lanelet::setLineMarkingLeft)
        .def_property("line_marking_right", &Lanelet::getLineMarkingRight, &Lanelet::setLineMarkingRight)
        .def_property("users_oneway", &Lanelet::getUsersOneWay, &Lanelet::setUsersOneWay)
        .def_property("users_bidirectional", &Lanelet::getUsersBidirectional, &Lanelet::setUsersBidirectional)
        .def_property("stop_line", &Lanelet::getStopLine, &Lanelet::setStopLine)

        .def_property_readonly("traffic_lights", &Lanelet::getTrafficLights)
        .def_property_readonly("traffic_signs", &Lanelet::getTrafficSigns)
        .def_property_readonly("outer_polygon", &Lanelet::getOuterPolygon);

    py::class_<Lane, Lanelet, std::shared_ptr<Lane>>(m, "Lane").def_property_readonly("contained_lanelets",
                                                                                      &Lane::getContainedLanelets);

    py::class_<RoadNetwork, std::shared_ptr<RoadNetwork>>(m, "RoadNetwork")
        .def_property_readonly("lanelets", &RoadNetwork::getLaneletNetwork)
        .def_property_readonly("lanes", &RoadNetwork::getLanes)
        .def_property_readonly("traffic_signs", &RoadNetwork::getTrafficSigns);

    py::class_<World, std::shared_ptr<World>>(m, "World")
        .def_property_readonly("time_step", &World::getTimeStep)
        .def_property_readonly("road_network", &World::getRoadNetwork)
        .def_property_readonly("obstacles", &World::getObstacles);

    m.def("create_world", &XMLReader::createWorldFromXML);
}