#include <memory>

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/occupancy.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "commonroad_cpp/geometry/circle.h"
#include "commonroad_cpp/geometry/polygon.h"
#include "commonroad_cpp/geometry/rectangle.h"
#include "commonroad_cpp/geometry/shape_group.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h"
#include "python_interface_core.h"
#include "python_interface_predicates.h"
#include "translate_python_types.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

static std::string extractName(nb::handle py_scen) {
    if (py_scen.attr("configuration_id").is_none() || py_scen.attr("obstacle_behavior").is_none() ||
        py_scen.attr("prediction_id").is_none())
        return {nb::cast<std::string>(py_scen.attr("country_id")) + "_" +
                nb::cast<std::string>(py_scen.attr("map_name")) + "-" +
                std::to_string(nb::cast<int>(py_scen.attr("map_id")))};
    else
        return {nb::cast<std::string>(py_scen.attr("country_id")) + "_" +
                nb::cast<std::string>(py_scen.attr("map_name")) + "-" +
                std::to_string(nb::cast<int>(py_scen.attr("map_id"))) + "_" +
                std::to_string(nb::cast<int>(py_scen.attr("configuration_id"))) + "_" +
                nb::cast<std::string>(py_scen.attr("obstacle_behavior")) + "-" +
                std::to_string(nb::cast<int>(py_scen.attr("prediction_id")))};
}

void updateTrajectory(Obstacle *t, const nb::handle &py_state_list) {
    tsl::robin_map<time_step_t, std::shared_ptr<State>> trajectory;
    for (const auto &py_state : py_state_list) {
        auto state = TranslatePythonTypes::extractState(py_state);
        trajectory[state->getTimeStep()] = state;
    }
    t->setTrajectoryPrediction(trajectory);
}

void updateCurrentState(Obstacle *t, const nb::handle &py_current_state) {
    t->updateCurrentState(TranslatePythonTypes::extractState(py_current_state));
}

void updateObstaclesTraj(World *t, const nb::list &py_obstacles, const nb::mapping &py_current_states,
                         const nb::mapping &py_trajectories) {
    std::map<size_t, std::shared_ptr<State>> currentStates;
    std::map<size_t, tsl::robin_map<time_step_t, std::shared_ptr<State>>> trajectoryPredictions;
    auto wp{t->getWorldParameters()};
    std::vector<std::shared_ptr<Obstacle>> obstacleList{
        TranslatePythonTypes::convertObstacles(py_obstacles, wp, false)};

    for (const auto &item : py_current_states.items()) {
        auto obsId{nb::cast<size_t>(item[0])};
        auto state{item[1]};
        currentStates[obsId] = TranslatePythonTypes::extractState(state);
    }

    for (const auto &item : py_trajectories.items()) {
        auto obsId{nb::cast<size_t>(item[0])};
        auto traj{item[1]};
        tsl::robin_map<time_step_t, std::shared_ptr<State>> trajectory;
        for (const auto &py_state : traj) {
            auto state = TranslatePythonTypes::extractState(py_state);
            trajectory[state->getTimeStep()] = state;
        }
        trajectoryPredictions[obsId] = trajectory;
    }
    t->updateObstaclesTraj(obstacleList, currentStates, trajectoryPredictions);
}

void updateObstacles(World *t, const nb::list &py_obstacles) {
    auto wp{t->getWorldParameters()};
    std::vector<std::shared_ptr<Obstacle>> obstacleList{
        TranslatePythonTypes::convertObstacles(py_obstacles, wp, false)};
    t->updateObstacles(obstacleList);
}

nb::dict getTrajectoryPrediction(Obstacle *t) {
    nb::dict trajDict;
    for (const auto &item : t->getTrajectoryPrediction()) {
        trajDict[nb::cast(item.first)] = nb::cast(item.second);
    }
    return trajDict;
}

nb::dict getSetBasedPrediction(Obstacle *t) {
    nb::dict predDict;
    for (const auto &item : t->getSetBasedPrediction()) {
        predDict[nb::cast(item.first)] = nb::cast(item.second);
    }
    return predDict;
}

nb::dict getTrajectoryHistory(Obstacle *t) {
    nb::dict trajDict;
    for (const auto &item : t->getTrajectoryHistory()) {
        trajDict[nb::cast(item.first)] = nb::cast(item.second);
    }
    return trajDict;
}

void init_python_interface_core(nb::module_ &m) {
    nb::enum_<ObstacleType>(m, "ObstacleType")
        .value("unknown", ObstacleType::unknown)
        .value("car", ObstacleType::car)
        .value("truck", ObstacleType::truck)
        .value("bus", ObstacleType::bus)
        .value("bicycle", ObstacleType::bicycle)
        .value("pedestrian", ObstacleType::pedestrian)
        .value("priority_vehicle", ObstacleType::priority_vehicle)
        .value("parked_vehicle", ObstacleType::parked_vehicle)
        .value("construction_zone", ObstacleType::construction_zone)
        .value("train", ObstacleType::train)
        .value("road_boundary", ObstacleType::road_boundary)
        .value("motorcycle", ObstacleType::motorcycle)
        .value("taxi", ObstacleType::taxi)
        .value("building", ObstacleType::building)
        .value("pillar", ObstacleType::pillar)
        .value("median_strip", ObstacleType::median_strip)
        .value("vehicle", ObstacleType::vehicle)
        .value("vru", ObstacleType::vru)
        .value("special_purpose_vehicle", ObstacleType::special_purpose_vehicle)
        .export_values();

    nb::enum_<LineMarking>(m, "LineMarking")
        .value("solid", LineMarking::solid)
        .value("dashed", LineMarking::dashed)
        .value("broad_dashed", LineMarking::broad_dashed)
        .value("broad_solid", LineMarking::broad_solid)
        .value("unknown", LineMarking::unknown)
        .value("no_marking", LineMarking::no_marking)
        .export_values();

    // TODO: Add missing lanelet types
    nb::enum_<LaneletType>(m, "LaneletType")
        .value("left", LaneletType::left)
        .value("right", LaneletType::right)
        .value("unknown", LaneletType::unknown)
        .export_values();

    nb::class_<StopLine>(m, "StopLine")
        .def(nb::init<>())
        .def(nb::init<std::pair<vertex, vertex>, LineMarking>())
        .def_prop_rw("points", &StopLine::getPoints, &StopLine::setPoints)
        .def_prop_rw("line_marking", &StopLine::getLineMarking, &StopLine::setLineMarking);

    nb::class_<Shape>(m, "Shape");

    nb::class_<Circle, Shape>(m, "Circle")
        .def(nb::init<double>())
        .def_prop_rw("radius", &Circle::getRadius, &Circle::setRadius);

    nb::class_<Rectangle, Shape>(m, "Rectangle")
        .def(nb::init<double, double>())
        .def_prop_rw("width", &Rectangle::getWidth, &Rectangle::setWidth)
        .def_prop_rw("length", &Rectangle::getLength, &Rectangle::setLength);

    nb::class_<Polygon, Shape>(m, "Polygon")
        .def(nb::init<std::vector<vertex>>())
        .def_prop_ro("vertices", &Polygon::getPolygonVertices);

    nb::class_<ShapeGroup, Shape>(m, "ShapeGroup")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::shared_ptr<Shape>>>())
        .def_prop_ro("shapes", &ShapeGroup::getShapes);

    nb::class_<Occupancy>(m, "Occupancy")
        .def(nb::init<>())
        .def(nb::init<size_t, std::shared_ptr<Shape>>())
        .def_prop_rw("time_step", &Occupancy::getTimeStep, &Occupancy::setTimeStep)
        .def_prop_rw("shape", &Occupancy::getShape, &Occupancy::setShape);

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
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double>())
        .def_prop_ro("v_max", &ActuatorParameters::getVmax)
        .def_prop_ro("a_max", &ActuatorParameters::getAmax)
        .def_prop_ro("a_max_long", &ActuatorParameters::getAmaxLong)
        .def_prop_ro("a_min_long", &ActuatorParameters::getAminLong)
        .def_prop_ro("a_braking", &ActuatorParameters::getAbraking)
        .def_static("ego_defaults", &ActuatorParameters::egoDefaults)
        .def_static("pedestrians_defaults", &ActuatorParameters::pedestrianDefaults)
        .def_static("static_defaults", &ActuatorParameters::staticDefaults)
        .def_static("vehicle_defaults", &ActuatorParameters::vehicleDefaults);

    nb::class_<SensorParameters>(m, "SensorParameters")
        .def(nb::init<>())
        .def(nb::init<double, double>())
        .def_prop_ro("fov_front", &SensorParameters::getFieldOfViewFront)
        .def_prop_ro("fov_rear", &SensorParameters::getFieldOfViewRear);

    nb::class_<TimeParameters>(m, "TimeParameters")
        .def(nb::init<>())
        .def(nb::init<double, double, double>())
        .def_prop_ro("relevant_history_size", &TimeParameters::getRelevantHistorySize)
        .def_prop_ro("time_step_size", &TimeParameters::getTimeStepSize)
        .def_prop_ro("reaction_time", &TimeParameters::getReactionTime);

    nb::class_<RoadNetworkParameters>(m, "RoadNetworkParameters")
        .def(nb::init<>())
        .def_rw("num_intersections_per_direction_lane_generation",
                &RoadNetworkParameters::numIntersectionsPerDirectionLaneGeneration);

    nb::class_<WorldParameters>(m, "WorldParameters")
        .def(
            nb::init<RoadNetworkParameters, SensorParameters, ActuatorParameters, TimeParameters, ActuatorParameters>())
        .def_prop_ro("road_network_params", &WorldParameters::getRoadNetworkParams)
        .def_prop_ro("sensor_params", &WorldParameters::getSensorParams)
        .def_prop_ro("actuator_params_ego", &WorldParameters::getActuatorParamsEgo)
        .def_prop_ro("actuator_params_obstacles", &WorldParameters::getActuatorParamsObstacles)
        .def_prop_ro("time_params", &WorldParameters::getTimeParams);

    nb::class_<vertex>(m, "Vertex").def(nb::init<>()).def_rw("x", &vertex::x).def_rw("y", &vertex::y);

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

                new (t) Obstacle(obs->getId(), obs->getObstacleRole(), obs->getCurrentState(), obs->getObstacleType(),
                                 obs->getVmax(), obs->getAmax(), obs->getAmaxLong(), obs->getAminLong(),
                                 obs->getReactionTime(), obs->getTrajectoryPrediction(), obs->getGeoShape().getLength(),
                                 obs->getGeoShape().getWidth());
            },
            "py_obstacle")
        .def_prop_rw("id", &Obstacle::getId, &Obstacle::setId)
        .def_prop_rw("type", &Obstacle::getObstacleType, &Obstacle::setObstacleType)
        .def_prop_ro("current_state", &Obstacle::getCurrentState)
        .def_prop_ro("current_signal_state", &Obstacle::getCurrentSignalState)
        .def_prop_ro("trajectory_prediction", &getTrajectoryPrediction)
        .def_prop_ro("history", &getTrajectoryHistory)
        .def_prop_ro("set_based_prediction", &getSetBasedPrediction)
        .def("shape", &Obstacle::getGeoShape)
        .def("occupied_lanes", &Obstacle::getOccupiedLanes)
        .def("occupied_lanelets", &Obstacle::getOccupiedLaneletsByShape)
        .def("driving_direction_lanes", &Obstacle::getOccupiedLanesDrivingDirection)
        .def("time_step_exists", &Obstacle::timeStepExists)
        .def_prop_rw("actuator_parameters", &Obstacle::getActuatorParameters, &Obstacle::setActuatorParameters)
        .def_prop_rw("sensor_parameters", &Obstacle::getSensorParameters, &Obstacle::setSensorParameters)
        .def_prop_rw("time_parameters", &Obstacle::getTimeParameters, &Obstacle::setTimeParameters)
        .def("get_state_by_time_step", &Obstacle::getStateByTimeStep)
        .def("reference_lane_by_time_step", &Obstacle::getReferenceLane)
        .def("get_time_steps", &Obstacle::getTimeSteps)
        .def("update_trajectory", &updateTrajectory, "py_state_list")
        .def("update_current_state", &updateCurrentState, "py_current_state");

    nb::class_<TrafficSign>(m, "TrafficSign").def_prop_rw("id", &TrafficSign::getId, &TrafficSign::setId);

    nb::class_<TrafficLight>(m, "TrafficLight").def_prop_rw("id", &TrafficLight::getId, &TrafficLight::setId);

    nb::class_<Lanelet>(m, "Lanelet")
        .def_prop_rw("id", &Lanelet::getId, &Lanelet::setId)
        .def_prop_rw("left_border_vertices", &Lanelet::getLeftBorderVertices, &Lanelet::setLeftBorderVertices)
        .def_prop_rw("right_border_vertices", &Lanelet::getRightBorderVertices, &Lanelet::setRightBorderVertices)
        .def_prop_ro("center_vertices", &Lanelet::getCenterVertices)
        .def_prop_rw("lanelet_types", &Lanelet::getLaneletTypes, &Lanelet::setLaneletTypes)
        .def_prop_rw("line_marking_left", &Lanelet::getLineMarkingLeft, &Lanelet::setLineMarkingLeft)
        .def_prop_rw("line_marking_right", &Lanelet::getLineMarkingRight, &Lanelet::setLineMarkingRight)
        .def_prop_rw("users_oneway", &Lanelet::getUsersOneWay, &Lanelet::setUsersOneWay)
        .def_prop_rw("users_bidirectional", &Lanelet::getUsersBidirectional, &Lanelet::setUsersBidirectional)
        .def_prop_rw("stop_line", &Lanelet::getStopLine, &Lanelet::setStopLine)
        .def_prop_ro("traffic_lights", &Lanelet::getTrafficLights)
        .def_prop_ro("traffic_signs", &Lanelet::getTrafficSigns)
        // TODO: Needs translation for boost::geometry::model::polygon
        // .def_prop_ro("outer_polygon", &Lanelet::getOuterPolygon)
        ;

    nb::class_<Lane, Lanelet>(m, "Lane").def_prop_ro("contained_lanelets", &Lane::getContainedLanelets);

    nb::class_<RoadNetwork>(m, "RoadNetwork")
        .def_prop_ro("lanelets", &RoadNetwork::getLaneletNetwork)
        .def_prop_ro("lanes", &RoadNetwork::getLanes)
        .def_prop_ro("traffic_signs", &RoadNetwork::getTrafficSigns);

    nb::class_<World>(m, "World")
        .def("__init__",
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

                 auto wp{WorldParameters()};
                 auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles, wp, false);
                 auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles, wp, true);

                 new (t) World(scenarioName, timeStep, roadNetwork, tempEgoVehicleContainer, tempObstacleContainer, dt);
             })
        .def("__init__",
             [](World *t, const std::string &scenarioName, size_t timeStep, double dt, const std::string &country,
                const nb::handle &py_laneletNetwork, const nb::list &py_egoVehicles, const nb::list &py_obstacles,
                WorldParameters wp) {
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

                 auto tempObstacleContainer = TranslatePythonTypes::convertObstacles(py_obstacles, wp, false);
                 auto tempEgoVehicleContainer = TranslatePythonTypes::convertObstacles(py_egoVehicles, wp, true);

                 new (t)
                     World(scenarioName, timeStep, roadNetwork, tempEgoVehicleContainer, tempObstacleContainer, dt, wp);
             })
        .def("__init__",
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

                 auto wp{WorldParameters()};
                 auto tempObstacleContainer =
                     TranslatePythonTypes::convertObstacles(py_scenario.attr("obstacles"), wp, false);

                 new (t) World(extractName(py_scenario.attr("scenario_id")), 0, roadNetwork, {}, tempObstacleContainer,
                               nb::cast<double>(py_scenario.attr("dt")));
             })
        .def("__init__",
             [](World *t, const nb::handle &py_scenario, WorldParameters wp) {
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

                 auto tempObstacleContainer =
                     TranslatePythonTypes::convertObstacles(py_scenario.attr("obstacles"), wp, false);

                 new (t) World(extractName(py_scenario.attr("scenario_id")), 0, roadNetwork, {}, tempObstacleContainer,
                               nb::cast<double>(py_scenario.attr("dt")), wp);
             })
        .def(nb::init<std::string, size_t, const std::shared_ptr<RoadNetwork> &, std::vector<std::shared_ptr<Obstacle>>,
                      std::vector<std::shared_ptr<Obstacle>>, double>())
        .def(nb::init<std::string, size_t, const std::shared_ptr<RoadNetwork> &, std::vector<std::shared_ptr<Obstacle>>,
                      std::vector<std::shared_ptr<Obstacle>>, double, WorldParameters>())
        .def_prop_ro("time_step", &World::getTimeStep)
        .def_prop_ro("road_network", &World::getRoadNetwork)
        .def_prop_ro("ego_vehicles", &World::getEgoVehicles)
        .def_prop_ro("obstacles", &World::getObstacles)
        .def("propagate", &World::propagate, nb::arg("ego") = true)
        .def("update_obstacles", &World::updateObstacles)
        .def("update_obstacles", &updateObstacles)
        .def("update_obstacles_traj", &updateObstaclesTraj);

    m.def("create_world", &XMLReader::createWorldFromXML);

    m.def("read_scenario", &InputUtils::getDataFromCommonRoad);

    init_python_interface_predicates(m);
}
