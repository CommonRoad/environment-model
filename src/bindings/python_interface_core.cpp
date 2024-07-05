#include <memory>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "python_interface_core.h"
#include "translate_python_types.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

static std::string extractName(nb::handle py_scen) {
    return {nb::cast<std::string>(py_scen.attr("country_id")) + "_" + nb::cast<std::string>(py_scen.attr("map_name")) +
            "-" + std::to_string(nb::cast<int>(py_scen.attr("map_id"))) + "_" +
            std::to_string(nb::cast<int>(py_scen.attr("configuration_id"))) + "_" +
            nb::cast<std::string>(py_scen.attr("obstacle_behavior")) + "-" +
            std::to_string(nb::cast<int>(py_scen.attr("prediction_id")))};
}

void init_python_interface_core(nb::module_ &m) {
    nb::enum_<ObstacleType>(m, "ObstacleType")
        .value("car", ObstacleType::car)
        .value("truck", ObstacleType::truck)
        .value("pedestrian", ObstacleType::pedestrian)
        .value("bus", ObstacleType::bus)
        .value("unknown", ObstacleType::unknown)
        .value("bicycle", ObstacleType::bicycle)
        .value("priority_vehicle", ObstacleType::priority_vehicle)
        .value("train", ObstacleType::train)
        .value("motorcycle", ObstacleType::motorcycle)
        .value("taxi", ObstacleType::taxi)
        .export_values();

    nb::enum_<LineMarking>(m, "LineMarking")
        .value("solid", LineMarking::solid)
        .value("dashed", LineMarking::dashed)
        .value("broad_dashed", LineMarking::broad_dashed)
        .value("broad_solid", LineMarking::broad_solid)
        .value("unknown", LineMarking::unknown)
        .value("no_marking", LineMarking::no_marking)
        .export_values();

    // nb::class_<Shape>(m, "Shape");

    nb::class_<State>(m, "State")
        .def_prop_rw("time_step", &State::getTimeStep, &State::setTimeStep)
        .def_prop_rw("x", &State::getXPosition, &State::setXPosition)
        .def_prop_rw("y", &State::getYPosition, &State::setYPosition)
        .def_prop_rw("lon", &State::getLonPosition, &State::setLonPosition)
        .def_prop_rw("lat", &State::getLatPosition, &State::setLatPosition)
        .def_prop_rw("global_orientation", &State::getGlobalOrientation, &State::setGlobalOrientation)
        .def_prop_rw("curvilinear_orientation", &State::getCurvilinearOrientation, &State::setCurvilinearOrientation)
        .def_prop_rw("velocity", &State::getVelocity, &State::setVelocity)
        .def_prop_rw("acceleration", &State::getAcceleration, &State::setAcceleration);

    nb::class_<SignalState>(m, "SignalState")
        .def_prop_rw("time_step", &SignalState::getTimeStep, &SignalState::setTimeStep)
        .def_prop_rw("horn", &SignalState::isHorn, &SignalState::setHorn)
        .def_prop_rw("indicator_left", &SignalState::isIndicatorLeft, &SignalState::setIndicatorLeft)
        .def_prop_rw("indicator_right", &SignalState::isIndicatorRight, &SignalState::setIndicatorRight)
        .def_prop_rw("brakingLights", &SignalState::isBrakingLights, &SignalState::setBrakingLights)
        .def_prop_rw("hazard_warning_lights", &SignalState::isHazardWarningLights, &SignalState::setHazardWarningLights)
        .def_prop_rw("flashing_blue_lights", &SignalState::isFlashingBlueLights, &SignalState::setFlashingBlueLights);

    nb::class_<ActuatorParameters>(m, "ActuatorParameters")
        .def_prop_ro("v_max", &ActuatorParameters::getVmax)
        .def_prop_ro("a_max", &ActuatorParameters::getAmax)
        .def_prop_ro("a_max_long", &ActuatorParameters::getAmaxLong)
        .def_prop_ro("a_min_long", &ActuatorParameters::getAminLong)
        .def_prop_ro("a_braking", &ActuatorParameters::getAbraking);

    nb::class_<SensorParameters>(m, "SensorParameters")
        .def_prop_ro("fov_front", &SensorParameters::getFieldOfViewFront)
        .def_prop_ro("fov_rear", &SensorParameters::getFieldOfViewRear)
        .def_prop_ro("reaction_time", &SensorParameters::getReactionTime);

    nb::class_<Obstacle>(m, "Obstacle")
        .def(
            "__init__",
            [](Obstacle *t, const nb::handle &py_obstacle) {
                std::string obstacleRole{nb::cast<std::string>(py_obstacle.attr("obstacle_role").attr("value"))};
                std::shared_ptr<Obstacle> obs;
                if (obstacleRole == "dynamic")
                    obs = TranslatePythonTypes::createDynamicObstacle(py_obstacle);
                else if (obstacleRole == "static")
                    obs = TranslatePythonTypes::createStaticObstacle(py_obstacle);
                std::vector<vertex> fov;
                for (const auto &vert : obs->getFov().outer())
                    fov.push_back({vert.x(), vert.y()});

                new (t) Obstacle(obs->getId(), obs->getObstacleRole(), obs->getCurrentState(), obs->getObstacleType(),
                                 obs->getVmax(), obs->getAmax(), obs->getAmaxLong(), obs->getAminLong(),
                                 obs->getReactionTime(), obs->getTrajectoryPrediction(), obs->getGeoShape().getLength(),
                                 obs->getGeoShape().getWidth(), fov);
            },
            "py_obstacle")
        .def_prop_rw("id", &Obstacle::getId, &Obstacle::setId)
        .def_prop_rw("type", &Obstacle::getObstacleType, &Obstacle::setObstacleType)
        .def_prop_ro("current_state", &Obstacle::getCurrentState)
        .def_prop_ro("current_signal_state", &Obstacle::getCurrentSignalState)
        .def("shape", &Obstacle::getGeoShape)
        .def("occupied_lanes", &Obstacle::getOccupiedLanes)
        .def("driving_direction_lanes", &Obstacle::getOccupiedLanesDrivingDirection)
        .def("time_step_exists", &Obstacle::timeStepExists)
        .def_prop_rw("actuator_parameters", &Obstacle::getActuatorParameters, &Obstacle::setActuatorParameters)
        .def_prop_rw("sensor_parameters", &Obstacle::getSensorParameters, &Obstacle::setSensorParameters)
        .def("get_state_by_time_step", &Obstacle::getStateByTimeStep)
        .def("reference_lane_by_time_step", &Obstacle::getReferenceLane)
        .def("get_time_steps", &Obstacle::getTimeSteps);

    // nb::class_<StopLine>(m, "StopLine");

    nb::class_<TrafficSign>(m, "TrafficSign").def_prop_rw("id", &TrafficSign::getId, &TrafficSign::setId);

    nb::class_<TrafficLight>(m, "TrafficLight").def_prop_rw("id", &TrafficLight::getId, &TrafficLight::setId);

    nb::class_<Lanelet>(m, "Lanelet")
        .def_prop_rw("id", &Lanelet::getId, &Lanelet::setId)
        .def_prop_rw("left_border_vertices", &Lanelet::getLeftBorderVertices, &Lanelet::setLeftBorderVertices)
        .def_prop_rw("right_border_vertices", &Lanelet::getRightBorderVertices, &Lanelet::setRightBorderVertices)
        .def_prop_rw("lanelet_types", &Lanelet::getLaneletTypes, &Lanelet::setLaneletTypes)
        .def_prop_rw("line_marking_left", &Lanelet::getLineMarkingLeft, &Lanelet::setLineMarkingLeft)
        .def_prop_rw("line_marking_right", &Lanelet::getLineMarkingRight, &Lanelet::setLineMarkingRight)
        .def_prop_rw("users_oneway", &Lanelet::getUsersOneWay, &Lanelet::setUsersOneWay)
        .def_prop_rw("users_bidirectional", &Lanelet::getUsersBidirectional, &Lanelet::setUsersBidirectional)
        .def_prop_rw("stop_line", &Lanelet::getStopLine, &Lanelet::setStopLine)
        .def_prop_ro("traffic_lights", &Lanelet::getTrafficLights)
        .def_prop_ro("traffic_signs", &Lanelet::getTrafficSigns)
        .def_prop_ro("outer_polygon", &Lanelet::getOuterPolygon);

    nb::class_<Lane, Lanelet>(m, "Lane").def_prop_ro("contained_lanelets", &Lane::getContainedLanelets);

    nb::class_<RoadNetwork>(m, "RoadNetwork")
        .def_prop_ro("lanelets", &RoadNetwork::getLaneletNetwork)
        .def_prop_ro("lanes", &RoadNetwork::getLanes)
        .def_prop_ro("traffic_signs", &RoadNetwork::getTrafficSigns);

    nb::class_<World>(m, "World")
        .def(
            "__init__",
            [](World *t, const std::string &scenarioName, size_t timeStep, double dt, const std::string &country,
               const nb::handle &py_laneletNetwork, const nb::list &py_egoVehicles, const nb::list &py_obstacles) {
                auto net{std::make_shared<RoadNetwork>(RoadNetwork({}))};
                auto convertedCountry{RoadNetwork::matchStringToCountry(country)};
                auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
                auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
                auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(
                    py_laneletNetwork, tempTrafficSignContainer, tempTrafficLightContainer);
                auto tempIntersectionContainer =
                    TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
                auto roadNetwork =
                    std::make_shared<RoadNetwork>(tempLaneletContainer, convertedCountry, tempTrafficSignContainer,
                                                  tempTrafficLightContainer, tempIntersectionContainer);
                for (const auto &intersection : roadNetwork->getIntersections())
                    intersection->computeMemberLanelets(roadNetwork);

                auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles);
                auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles);

                new (t) World(scenarioName, timeStep, roadNetwork, tempEgoVehicleContainer, tempObstacleContainer, dt);
            },
            "scenarioName", "timeStep", "dt", "country", "py_laneletNetwork", "py_egoVehicles", "py_obstacles")
        .def(
            "__init__",
            [](World *t, const nb::handle &py_scenario) {
                auto py_laneletNetwork{py_scenario.attr("lanelet_network")};
                std::string country{nb::cast<std::string>(py_scenario.attr("scenario_id").attr("country_id"))};

                auto net{std::make_shared<RoadNetwork>(RoadNetwork({}))};
                auto convertedCountry{RoadNetwork::matchStringToCountry(country)};
                auto tempTrafficSignContainer = TranslatePythonTypes::convertTrafficSigns(py_laneletNetwork);
                auto tempTrafficLightContainer = TranslatePythonTypes::convertTrafficLights(py_laneletNetwork);
                auto tempLaneletContainer = TranslatePythonTypes::convertLanelets(
                    py_laneletNetwork, tempTrafficSignContainer, tempTrafficLightContainer);
                auto tempIntersectionContainer =
                    TranslatePythonTypes::convertIntersections(py_laneletNetwork, tempLaneletContainer);
                auto roadNetwork =
                    std::make_shared<RoadNetwork>(tempLaneletContainer, convertedCountry, tempTrafficSignContainer,
                                                  tempTrafficLightContainer, tempIntersectionContainer);
                for (const auto &intersection : roadNetwork->getIntersections())
                    intersection->computeMemberLanelets(roadNetwork);

                auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_scenario.attr("obstacles"));

                new (t) World(extractName(py_scenario.attr("scenario_id")), 0, roadNetwork, {}, tempObstacleContainer,
                              nb::cast<double>(py_scenario.attr("dt")));
            },
            "py_scenario")
        .def_prop_ro("time_step", &World::getTimeStep)
        .def_prop_ro("road_network", &World::getRoadNetwork)
        .def_prop_ro("obstacles", &World::getObstacles);

    m.def("create_world", &XMLReader::createWorldFromXML);
}
