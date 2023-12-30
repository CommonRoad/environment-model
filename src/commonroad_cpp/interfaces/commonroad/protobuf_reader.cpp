//
// Created by Yannick Ballnath.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/interfaces/commonroad/protobuf_reader.h"
#include <stdexcept>
#include <utility>

commonroad::CommonRoad ProtobufReader::loadProtobufMessage(const std::string &filePath)
// Loads a CommonRoad message from protobuf file
{
    std::ifstream pbFile(filePath, std::ios::binary | std::ios::ate);
    std::streamsize size = pbFile.tellg();
    pbFile.seekg(0, std::ios::beg);

    std::vector<char> buffer((size_t)size);
    pbFile.read(buffer.data(), size);

    std::string commonRoadStr(buffer.begin(), buffer.end());

    commonroad::CommonRoad commonRoadMsg = commonroad::CommonRoad();
    commonRoadMsg.ParseFromString(commonRoadStr);

    return commonRoadMsg;
}

void ProtobufReader::initLaneletContainer(ProtobufReader::LaneletContainer &laneletContainer,
                                          const commonroad::CommonRoad &commonRoadMsg) {
    for (const auto &laneletMsg : commonRoadMsg.lanelets())
        laneletContainer.emplace(laneletMsg.lanelet_id(), std::make_shared<Lanelet>());
}

void ProtobufReader::initTrafficSignContainer(ProtobufReader::TrafficSignContainer &trafficSignContainer,
                                              const commonroad::CommonRoad &commonRoadMsg) {
    for (const auto &trafficSignMsg : commonRoadMsg.traffic_signs())
        trafficSignContainer.emplace(trafficSignMsg.traffic_sign_id(), std::make_shared<TrafficSign>());
}

void ProtobufReader::initTrafficLightContainer(ProtobufReader::TrafficLightContainer &trafficLightContainer,
                                               const commonroad::CommonRoad &commonRoadMsg) {
    for (const auto &trafficLightMsg : commonRoadMsg.traffic_lights())
        trafficLightContainer.emplace(trafficLightMsg.traffic_light_id(), std::make_shared<TrafficLight>());
}

void ProtobufReader::initIncomingContainer(IncomingContainer &incomingContainer,
                                           const commonroad::Intersection &intersectionMsg) {
    for (const auto &incomingMsg : intersectionMsg.incomings())
        incomingContainer.emplace(incomingMsg.incoming_id(), std::make_shared<Incoming>());
}

std::shared_ptr<Lanelet> ProtobufReader::getLaneletFromContainer(size_t laneletId,
                                                                 ProtobufReader::LaneletContainer &laneletContainer) {
    if (laneletContainer.find(laneletId) != laneletContainer.end())
        return laneletContainer[laneletId];

    return nullptr;
}

std::shared_ptr<TrafficSign>
ProtobufReader::getTrafficSignFromContainer(size_t trafficSignId,
                                            ProtobufReader::TrafficSignContainer &trafficSignContainer) {
    if (trafficSignContainer.find(trafficSignId) != trafficSignContainer.end())
        return trafficSignContainer[trafficSignId];

    return nullptr;
}

std::shared_ptr<TrafficLight>
ProtobufReader::getTrafficLightFromContainer(size_t trafficLightId,
                                             ProtobufReader::TrafficLightContainer &trafficLightContainer) {
    if (trafficLightContainer.find(trafficLightId) != trafficLightContainer.end())
        return trafficLightContainer[trafficLightId];

    return nullptr;
}

std::shared_ptr<Incoming>
ProtobufReader::getIncomingFromContainer(size_t incomingId, ProtobufReader::IncomingContainer &incomingContainer) {
    if (incomingContainer.find(incomingId) != incomingContainer.end())
        return incomingContainer[incomingId];

    return nullptr;
}

std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>
ProtobufReader::createCommonRoadFromMessage(const commonroad::CommonRoad &commonRoadMsg) {
    LaneletContainer laneletContainer;
    initLaneletContainer(laneletContainer, commonRoadMsg);

    TrafficSignContainer trafficSignContainer;
    initTrafficSignContainer(trafficSignContainer, commonRoadMsg);

    TrafficLightContainer trafficLightContainer;
    initTrafficLightContainer(trafficLightContainer, commonRoadMsg);

    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &laneletMsg : commonRoadMsg.lanelets())
        lanelets.push_back(ProtobufReader::createLaneletFromMessage(laneletMsg, laneletContainer, trafficSignContainer,
                                                                    trafficLightContainer));

    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;
    for (const auto &trafficSignMsg : commonRoadMsg.traffic_signs())
        trafficSigns.push_back(ProtobufReader::createTrafficSignFromMessage(trafficSignMsg, trafficSignContainer));

    std::vector<std::shared_ptr<TrafficLight>> trafficLights;
    for (const auto &trafficLightMsg : commonRoadMsg.traffic_lights())
        trafficLights.push_back(ProtobufReader::createTrafficLightFromMessage(trafficLightMsg, trafficLightContainer));

    std::vector<std::shared_ptr<Intersection>> intersections;
    for (const auto &intersectionMsg : commonRoadMsg.intersections())
        intersections.push_back(ProtobufReader::createIntersectionFromMessage(intersectionMsg, laneletContainer));

    std::vector<std::shared_ptr<Obstacle>> obstacles;
    for (const auto &dynamicObstacleMsg : commonRoadMsg.dynamic_obstacles())
        obstacles.push_back(ProtobufReader::createDynamicObstacleFromMessage(dynamicObstacleMsg));

    for (const auto &staticObstacleMsg : commonRoadMsg.static_obstacles())
        obstacles.push_back(ProtobufReader::createStaticObstacleFromMessage(staticObstacleMsg));

    for (const auto &environmentObstacleMsg : commonRoadMsg.environment_obstacles())
        obstacles.push_back(ProtobufReader::createEnvironmentObstacleFromMessage(environmentObstacleMsg));

    for (const auto &phantomObstacleMsg : commonRoadMsg.phantom_obstacles())
        obstacles.push_back(ProtobufReader::createPhantomObstacleFromMessage(phantomObstacleMsg));

    auto [benchmarkId, timeStepSize] =
        ProtobufReader::createScenarioInformationFromMessage(commonRoadMsg.information());

    std::string countryIdName = benchmarkId.substr(0, benchmarkId.find('_'));
    SupportedTrafficSignCountry countryId = RoadNetwork::matchStringToCountry(countryIdName);

    std::shared_ptr<RoadNetwork> roadNetwork =
        std::make_shared<RoadNetwork>(lanelets, countryId, trafficSigns, trafficLights, intersections);

    return std::make_tuple(obstacles, roadNetwork, timeStepSize);
}

std::tuple<std::string, double>
ProtobufReader::createScenarioInformationFromMessage(const commonroad::ScenarioInformation &scenarioInformationMsg) {
    std::string benchmarkId = scenarioInformationMsg.benchmark_id();

    double timeStepSize = scenarioInformationMsg.time_step_size();

    return std::make_tuple(benchmarkId, timeStepSize);
}

std::shared_ptr<Lanelet> ProtobufReader::createLaneletFromMessage(const commonroad::Lanelet &laneletMsg,
                                                                  LaneletContainer &laneletContainer,
                                                                  TrafficSignContainer &trafficSignContainer,
                                                                  TrafficLightContainer &trafficLightContainer) {
    std::shared_ptr<Lanelet> lanelet = laneletContainer[laneletMsg.lanelet_id()];

    lanelet->setId(laneletMsg.lanelet_id());

    auto [leftVertices, leftLineMarking] = ProtobufReader::createBoundFromMessage(laneletMsg.left_bound());
    auto [rightVertices, rightLineMarking] = ProtobufReader::createBoundFromMessage(laneletMsg.right_bound());

    lanelet->setLeftBorderVertices(leftVertices);

    lanelet->setRightBorderVertices(rightVertices);

    for (size_t laneletId : laneletMsg.successors()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->addSuccessor(getLaneletFromContainer(laneletId, laneletContainer));
    }

    for (size_t laneletId : laneletMsg.predecessors()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->addPredecessor(containerLanelet);
    }

    std::set<LaneletType> laneletTypes;
    for (const auto &laneletType : laneletMsg.lanelet_types()) {
        std::string laneletTypeName =
            commonroad::LaneletTypeEnum_LaneletType_Name((commonroad::LaneletTypeEnum_LaneletType)laneletType);
        laneletTypes.emplace(lanelet_operations::matchStringToLaneletType(laneletTypeName));
    }
    lanelet->setLaneletTypes(laneletTypes);

    std::set<ObstacleType> userOneWays;
    for (const auto &userOneWay : laneletMsg.user_one_way()) {
        std::string userOneWayName =
            commonroad::ObstacleTypeEnum_ObstacleType_Name((commonroad::ObstacleTypeEnum_ObstacleType)userOneWay);
        userOneWays.emplace(obstacle_operations::matchStringToObstacleType(userOneWayName));
    }
    lanelet->setUsersOneWay(userOneWays);

    std::set<ObstacleType> usersBidirectionals;
    for (const auto &usersBidirectional : laneletMsg.user_bidirectional()) {
        std::string userBidirectionalName = commonroad::ObstacleTypeEnum_ObstacleType_Name(
            (commonroad::ObstacleTypeEnum_ObstacleType)usersBidirectional);
        usersBidirectionals.emplace(obstacle_operations::matchStringToObstacleType(userBidirectionalName));
    }
    lanelet->setUsersBidirectional(usersBidirectionals);

    for (size_t vertex_i = 0; vertex_i < leftVertices.size(); vertex_i++)
        lanelet->addCenterVertex((leftVertices[vertex_i] + rightVertices[vertex_i]) / 2.);

    if (leftLineMarking != nullptr)
        lanelet->setLineMarkingLeft(*leftLineMarking.get());

    if (rightLineMarking != nullptr)
        lanelet->setLineMarkingRight(*rightLineMarking.get());

    DrivingDirection leftAdjacentDir = DrivingDirection::invalid;
    if (laneletMsg.has_adjacent_left_dir()) {
        std::string drivingDirectionName = commonroad::DrivingDirEnum_DrivingDir_Name(
            (commonroad::DrivingDirEnum_DrivingDir)laneletMsg.adjacent_left_dir());
        leftAdjacentDir = lanelet_operations::matchStringToDrivingDirection(drivingDirectionName);
    }

    DrivingDirection rightAdjacentDir = DrivingDirection::invalid;
    if (laneletMsg.has_adjacent_right_dir()) {
        std::string drivingDirectionName = commonroad::DrivingDirEnum_DrivingDir_Name(
            (commonroad::DrivingDirEnum_DrivingDir)laneletMsg.adjacent_right_dir());
        rightAdjacentDir = lanelet_operations::matchStringToDrivingDirection(drivingDirectionName);
    }

    if (laneletMsg.has_adjacent_left()) {
        auto containerLanelet = getLaneletFromContainer(laneletMsg.adjacent_left(), laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->setLeftAdjacent(containerLanelet, leftAdjacentDir);
    }

    if (laneletMsg.has_adjacent_right()) {
        auto containerLanelet = getLaneletFromContainer(laneletMsg.adjacent_right(), laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->setRightAdjacent(containerLanelet, rightAdjacentDir);
    }

    if (laneletMsg.has_stop_line())
        lanelet->setStopLine(ProtobufReader::createStopLineFromMessage(laneletMsg.stop_line(), trafficSignContainer,
                                                                       trafficLightContainer));

    for (size_t trafficSignId : laneletMsg.traffic_sign_refs()) {
        auto containerTrafficSign = getTrafficSignFromContainer(trafficSignId, trafficSignContainer);
        if (containerTrafficSign != nullptr)
            lanelet->addTrafficSign(containerTrafficSign);
    }

    for (size_t trafficLightId : laneletMsg.traffic_light_refs()) {
        auto containerTrafficLight = getTrafficLightFromContainer(trafficLightId, trafficLightContainer);
        if (containerTrafficLight != nullptr)
            lanelet->addTrafficLight(containerTrafficLight);
    }

    lanelet->constructOuterPolygon();
    return lanelet;
}

std::tuple<std::vector<vertex>, std::unique_ptr<LineMarking>>
ProtobufReader::createBoundFromMessage(const commonroad::Bound &boundMsg) {
    std::vector<vertex> points;
    for (const auto &pointMsg : boundMsg.points())
        points.push_back(ProtobufReader::createPointFromMessage(pointMsg));

    std::string lineMarkingName = commonroad::LineMarkingEnum_LineMarking_Name(boundMsg.line_marking());
    std::unique_ptr<LineMarking> lineMarking =
        std::make_unique<LineMarking>(lanelet_operations::matchStringToLineMarking(lineMarkingName));

    return std::make_tuple(points, std::move(lineMarking));
}

std::shared_ptr<StopLine> ProtobufReader::createStopLineFromMessage(const commonroad::StopLine &stopLineMsg,
                                                                    TrafficSignContainer &trafficSignContainer,
                                                                    TrafficLightContainer &trafficLightContainer) {
    std::shared_ptr<StopLine> stopLine = std::make_shared<StopLine>();

    std::vector<vertex> points;
    for (const auto &point : stopLineMsg.points())
        points.push_back(ProtobufReader::createPointFromMessage(point));
    stopLine->setPoints(points);

    std::string lineMarkingName = commonroad::LineMarkingEnum_LineMarking_Name(stopLineMsg.line_marking());
    LineMarking lineMarking = lanelet_operations::matchStringToLineMarking(lineMarkingName);
    stopLine->setLineMarking(lineMarking);

    for (size_t trafficSignId : stopLineMsg.traffic_sign_refs()) {
        auto containerTrafficSign = getTrafficSignFromContainer(trafficSignId, trafficSignContainer);
        if (containerTrafficSign != nullptr)
            stopLine->addTrafficSign(containerTrafficSign);
    }

    for (size_t trafficLightId : stopLineMsg.traffic_light_refs()) {
        auto containerTrafficLight = getTrafficLightFromContainer(trafficLightId, trafficLightContainer);
        if (containerTrafficLight != nullptr)
            stopLine->addTrafficLight(containerTrafficLight);
    }

    return stopLine;
}

std::shared_ptr<TrafficSign> ProtobufReader::createTrafficSignFromMessage(const commonroad::TrafficSign &trafficSignMsg,
                                                                          TrafficSignContainer &trafficSignContainer) {
    std::shared_ptr<TrafficSign> trafficSign = trafficSignContainer[trafficSignMsg.traffic_sign_id()];

    trafficSign->setId((size_t)trafficSignMsg.traffic_sign_id());

    std::vector<std::shared_ptr<TrafficSignElement>> trafficSignElements;
    for (const auto &trafficSignElementMsg : trafficSignMsg.traffic_sign_elements())
        trafficSignElements.push_back(ProtobufReader::createTrafficSignElementFromMessage(trafficSignElementMsg));
    trafficSign->setTrafficSignElement(trafficSignElements);

    if (trafficSignMsg.has_position())
        trafficSign->setPosition(ProtobufReader::createPointFromMessage(trafficSignMsg.position()));

    if (trafficSignMsg.has_virtual_())
        trafficSign->setVirtualElement(trafficSign->isVirtualElement());

    return trafficSign;
}

std::string mapGermanTrafficSignID(const std::string &signName) {
    // temporary quick-fix for mismatch in xml and protobuf names
    if (signName == "MAX_SPEED")
        return "274";
    else if (signName == "TOWN_SIGN")
        return "310";
    else if (signName == "PRIORITY")
        return "306";
    else if (signName == "GREEN_ARROW")
        return "720";
    else if (signName == "YIELD")
        return "205";
    else if (signName == "U_TURN")
        return "272";
    else if (signName.empty())
        return "274";
    throw std::runtime_error{"ProtobufReader::mapGermanTrafficSignID: incomplete, called for " + signName};
}

std::shared_ptr<TrafficSignElement>
ProtobufReader::createTrafficSignElementFromMessage(const commonroad::TrafficSignElement &trafficSignElementMsg) {
    std::shared_ptr<TrafficSignElement> trafficSignElement = std::make_shared<TrafficSignElement>();

    std::string trafficSignId;
    if (trafficSignElementMsg.has_germany_element_id())
        trafficSignId =
            commonroad::TrafficSignIDGermanyEnum_TrafficSignIDGermany_Name(trafficSignElementMsg.germany_element_id());
    else if (trafficSignElementMsg.has_zamunda_element_id())
        trafficSignId =
            commonroad::TrafficSignIDZamundaEnum_TrafficSignIDZamunda_Name(trafficSignElementMsg.zamunda_element_id());
    else if (trafficSignElementMsg.has_usa_element_id())
        trafficSignId = commonroad::TrafficSignIDUsaEnum_TrafficSignIDUsa_Name(trafficSignElementMsg.usa_element_id());
    else if (trafficSignElementMsg.has_spain_element_id())
        trafficSignId =
            commonroad::TrafficSignIDSpainEnum_TrafficSignIDSpain_Name(trafficSignElementMsg.spain_element_id());
    trafficSignElement->setId(
        mapGermanTrafficSignID(trafficSignId)); // TODO remove mapGermanTrafficSignID with new CommonRoad format

    std::vector<std::string> additionalValues(trafficSignElementMsg.additional_values().begin(),
                                              trafficSignElementMsg.additional_values().end());
    trafficSignElement->setAdditionalValues(additionalValues);

    return trafficSignElement;
}

std::shared_ptr<TrafficLight>
ProtobufReader::createTrafficLightFromMessage(const commonroad::TrafficLight &trafficLightMsg,
                                              TrafficLightContainer &trafficLightContainer) {
    std::shared_ptr<TrafficLight> trafficLight = trafficLightContainer[trafficLightMsg.traffic_light_id()];

    trafficLight->setId(trafficLightMsg.traffic_light_id());

    std::vector<TrafficLightCycleElement> cycleElements;
    for (const auto &cycleElementMsg : trafficLightMsg.cycle_elements())
        cycleElements.push_back(ProtobufReader::createCycleElementFromMessage(cycleElementMsg));
    trafficLight->setCycle(cycleElements);

    if (trafficLightMsg.has_position())
        trafficLight->setPosition(ProtobufReader::createPointFromMessage(trafficLightMsg.position()));

    if (trafficLightMsg.has_time_offset())
        trafficLight->setOffset(trafficLightMsg.time_offset());

    if (trafficLightMsg.has_direction()) {
        std::string directionName =
            commonroad::TrafficLightDirectionEnum_TrafficLightDirection_Name(trafficLightMsg.direction());
        trafficLight->setDirection(TrafficLight::matchTurningDirections(directionName));
    }

    if (trafficLightMsg.has_active())
        trafficLight->setActive(trafficLightMsg.active());

    return trafficLight;
}

TrafficLightCycleElement
ProtobufReader::createCycleElementFromMessage(const commonroad::CycleElement &cycleElementMsg) {
    TrafficLightCycleElement cycleElement;

    std::string trafficLightStateName =
        commonroad::TrafficLightStateEnum_TrafficLightState_Name(cycleElementMsg.color());
    cycleElement.color = TrafficLight::matchTrafficLightState(trafficLightStateName);

    cycleElement.duration = cycleElementMsg.duration();

    return cycleElement;
}

std::shared_ptr<Intersection>
ProtobufReader::createIntersectionFromMessage(const commonroad::Intersection &intersectionMsg,
                                              LaneletContainer &laneletContainer) {
    std::shared_ptr<Intersection> intersection = std::make_shared<Intersection>();

    intersection->setId(intersectionMsg.intersection_id());

    IncomingContainer incomingContainer;
    initIncomingContainer(incomingContainer, intersectionMsg);

    for (const auto &incomingMsg : intersectionMsg.incomings())
        intersection->addIncoming(
            ProtobufReader::createIncomingFromMessage(incomingMsg, laneletContainer, incomingContainer));

    for (size_t laneletId : intersectionMsg.crossing_lanelets()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            intersection->addCrossing(containerLanelet);
    }

    return intersection;
}

std::shared_ptr<Incoming> ProtobufReader::createIncomingFromMessage(const commonroad::Incoming &incomingMsg,
                                                                    LaneletContainer &laneletContainer,
                                                                    IncomingContainer &incomingContainer) {
    std::shared_ptr<Incoming> incoming = incomingContainer[incomingMsg.incoming_id()];

    incoming->setId(incomingMsg.incoming_id());

    for (size_t laneletId : incomingMsg.incoming_lanelets()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addIncomingLanelet(containerLanelet);
    }

    for (size_t laneletId : incomingMsg.successors_straight()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addStraightOutgoing(containerLanelet);
    }

    for (size_t laneletId : incomingMsg.successors_left()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addLeftOutgoing(containerLanelet);
    }

    for (size_t laneletId : incomingMsg.successors_right()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addRightOutgoing(containerLanelet);
    }

    if (incomingMsg.has_is_left_of()) {
        auto containerIncoming = getIncomingFromContainer(incomingMsg.is_left_of(), incomingContainer);
        if (containerIncoming != nullptr)
            incoming->setIsLeftOf(containerIncoming);
    }

    return incoming;
}

std::shared_ptr<Obstacle>
ProtobufReader::createStaticObstacleFromMessage(const commonroad::StaticObstacle &staticObstacleMsg) {
    std::shared_ptr<Obstacle> staticObstacle = std::make_shared<Obstacle>();

    staticObstacle->setId(staticObstacleMsg.static_obstacle_id());

    staticObstacle->setObstacleRole(ObstacleRole::STATIC);

    staticObstacle->setIsStatic(true);

    staticObstacle->setCurrentState(ProtobufReader::createStateFromMessage(staticObstacleMsg.initial_state()));

    std::string obstacleTypeName = commonroad::ObstacleTypeEnum_ObstacleType_Name(staticObstacleMsg.obstacle_type());
    staticObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(obstacleTypeName));

    staticObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(staticObstacleMsg.shape()));

    return staticObstacle;
}

std::shared_ptr<Obstacle>
ProtobufReader::createDynamicObstacleFromMessage(const commonroad::DynamicObstacle &dynamicObstacleMsg) {
    std::shared_ptr<Obstacle> dynamicObstacle = std::make_shared<Obstacle>();

    dynamicObstacle->setActuatorParameters(ActuatorParameters::vehicleDefaults());
    dynamicObstacle->setSensorParameters(SensorParameters::dynamicDefaults());

    dynamicObstacle->setId(dynamicObstacleMsg.dynamic_obstacle_id());

    dynamicObstacle->setObstacleRole(ObstacleRole::DYNAMIC);

    dynamicObstacle->setIsStatic(false);

    std::string obstacleTypeName = commonroad::ObstacleTypeEnum_ObstacleType_Name(dynamicObstacleMsg.obstacle_type());
    dynamicObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(obstacleTypeName));

    dynamicObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(dynamicObstacleMsg.shape()));

    dynamicObstacle->setCurrentState(ProtobufReader::createStateFromMessage(dynamicObstacleMsg.initial_state()));

    if (dynamicObstacleMsg.has_initial_signal_state())
        dynamicObstacle->setCurrentSignalState(
            ProtobufReader::createSignalStateFromMessage(dynamicObstacleMsg.initial_signal_state()));
    if (dynamicObstacleMsg.has_trajectory_prediction())
        for (const auto &state :
             ProtobufReader::createTrajectoryPredictionFromMessage(dynamicObstacleMsg.trajectory_prediction()))
            dynamicObstacle->appendStateToTrajectoryPrediction(state);
    if (!dynamicObstacleMsg.signal_series().empty())
        for (const auto &state : dynamicObstacleMsg.signal_series())
            dynamicObstacle->appendSignalStateToSeries(ProtobufReader::createSignalStateFromMessage(state));

    return dynamicObstacle;
}

std::shared_ptr<Obstacle>
ProtobufReader::createEnvironmentObstacleFromMessage(const commonroad::EnvironmentObstacle &environmentObstacleMsg) {
    std::shared_ptr<Obstacle> environmentObstacle = std::make_shared<Obstacle>();

    environmentObstacle->setId(environmentObstacleMsg.environment_obstacle_id());

    environmentObstacle->setObstacleRole(ObstacleRole::ENVIRONMENT);

    std::string obstacleTypeName =
        commonroad::ObstacleTypeEnum_ObstacleType_Name(environmentObstacleMsg.obstacle_type());
    environmentObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(obstacleTypeName));

    environmentObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(environmentObstacleMsg.obstacle_shape()));

    return environmentObstacle;
}

std::shared_ptr<Obstacle>
ProtobufReader::createPhantomObstacleFromMessage(const commonroad::PhantomObstacle &phantomObstacleMsg) {
    std::shared_ptr<Obstacle> phantomObstacle = std::make_shared<Obstacle>();

    phantomObstacle->setId(phantomObstacleMsg.obstacle_id());

    phantomObstacle->setObstacleRole(ObstacleRole::PHANTOM);

    // TODO: obstacle role and prediction

    return phantomObstacle;
}

std::vector<std::shared_ptr<State>>
ProtobufReader::createTrajectoryPredictionFromMessage(const commonroad::TrajectoryPrediction &trajectoryPredictionMsg) {
    return ProtobufReader::createTrajectoryFromMessage(trajectoryPredictionMsg.trajectory());
}

std::vector<std::shared_ptr<State>>
ProtobufReader::createTrajectoryFromMessage(const commonroad::Trajectory &trajectoryMsg) {
    std::vector<std::shared_ptr<State>> trajectory;

    for (const auto &stateMsg : trajectoryMsg.states())
        trajectory.push_back(ProtobufReader::createStateFromMessage(stateMsg));

    return trajectory;
}

std::shared_ptr<State> ProtobufReader::createStateFromMessage(const commonroad::State &stateMsg) {
    std::shared_ptr<State> state = std::make_shared<State>();

    IntegerExactOrInterval timeStep = ProtobufReader::createIntegerExactOrInterval(stateMsg.time_step());
    if (timeStep.index() == 0)
        state->setTimeStep((size_t)std::get<int>(timeStep));
    else
        throw std::logic_error("Type of time step is not supported.");

    if (stateMsg.has_point()) {
        vertex vertex = ProtobufReader::createPointFromMessage(stateMsg.point());
        state->setXPosition(vertex.x);
        state->setYPosition(vertex.y);
    }

    if (stateMsg.has_orientation()) {
        FloatExactOrInterval orientation = ProtobufReader::createFloatExactOrInterval(stateMsg.orientation());
        if (orientation.index() == 0)
            state->setGlobalOrientation(std::get<double>(orientation));
    }

    if (stateMsg.has_velocity()) {
        FloatExactOrInterval velocity = ProtobufReader::createFloatExactOrInterval(stateMsg.velocity());
        if (velocity.index() == 0)
            state->setVelocity(std::get<double>(velocity));
    }

    if (stateMsg.has_acceleration()) {
        FloatExactOrInterval acceleration = ProtobufReader::createFloatExactOrInterval(stateMsg.acceleration());
        if (acceleration.index() == 0)
            state->setAcceleration(std::get<double>(acceleration));
    }

    return state;
}

std::shared_ptr<SignalState> ProtobufReader::createSignalStateFromMessage(const commonroad::SignalState &stateMsg) {
    std::shared_ptr<SignalState> state = std::make_shared<SignalState>();

    if (stateMsg.has_time_step()) {
        IntegerExactOrInterval timeStep = ProtobufReader::createIntegerExactOrInterval(stateMsg.time_step());
        if (timeStep.index() == 0)
            state->setTimeStep((size_t)std::get<int>(timeStep));
    }
    if (stateMsg.has_horn())
        state->setHorn(true);
    if (stateMsg.indicator_left())
        state->setIndicatorLeft(true);
    if (stateMsg.indicator_right())
        state->setIndicatorRight(true);
    if (stateMsg.has_braking_lights())
        state->setBrakingLights(true);
    if (stateMsg.has_hazard_warning_lights())
        state->setHazardWarningLights(true);
    if (stateMsg.has_flashing_blue_lights())
        state->setFlashingBlueLights(true);

    return state;
}

vertex ProtobufReader::createPointFromMessage(const commonroad::Point &pointMsg) {
    vertex vertex;

    vertex.x = pointMsg.x();

    vertex.y = pointMsg.y();

    return vertex;
}

std::unique_ptr<Shape> ProtobufReader::createShapeFromMessage(const commonroad::Shape &shapeMsg) {
    std::unique_ptr<Shape> shape;

    if (shapeMsg.has_rectangle())
        shape = ProtobufReader::createRectangleFromMessage(shapeMsg.rectangle());
    else if (shapeMsg.has_circle())
        shape = ProtobufReader::createCircleFromMessage(shapeMsg.circle());

    return shape;
}

std::unique_ptr<Rectangle> ProtobufReader::createRectangleFromMessage(const commonroad::Rectangle &rectangleMsg) {
    std::unique_ptr<Rectangle> rectangle = std::make_unique<Rectangle>();

    rectangle->setLength(rectangleMsg.length());

    rectangle->setWidth(rectangleMsg.width());

    if (rectangleMsg.has_center()) {
        vertex point = ProtobufReader::createPointFromMessage(rectangleMsg.center());
        rectangle->setCenter(point.x, point.y);
    }

    return rectangle;
}

std::unique_ptr<Circle> ProtobufReader::createCircleFromMessage(const commonroad::Circle &circleMsg) {
    std::unique_ptr<Circle> circle = std::make_unique<Circle>();

    circle->setRadius(circleMsg.radius());

    if (circleMsg.has_center()) {
        vertex point = ProtobufReader::createPointFromMessage(circleMsg.center());
        circle->setCenter(point.x, point.y);
    }

    return circle;
}

ProtobufReader::IntegerInterval
ProtobufReader::createIntegerIntervalFromMessage(const commonroad::IntegerInterval &integerIntervalMsg) {
    return std::make_pair(integerIntervalMsg.start(), integerIntervalMsg.end());
}

ProtobufReader::FloatInterval
ProtobufReader::createFloatIntervalFromMessage(const commonroad::FloatInterval &floatIntervalMsg) {
    return std::make_pair(floatIntervalMsg.start(), floatIntervalMsg.end());
}

ProtobufReader::IntegerExactOrInterval
ProtobufReader::createIntegerExactOrInterval(const commonroad::IntegerExactOrInterval &integerExactOrIntervalMsg) {
    IntegerExactOrInterval integerExactOrInterval;

    if (integerExactOrIntervalMsg.has_exact())
        integerExactOrInterval = integerExactOrIntervalMsg.exact();
    else if (integerExactOrIntervalMsg.has_interval())
        integerExactOrInterval = ProtobufReader::createIntegerIntervalFromMessage(integerExactOrIntervalMsg.interval());

    return integerExactOrInterval;
}

ProtobufReader::FloatExactOrInterval
ProtobufReader::createFloatExactOrInterval(const commonroad::FloatExactOrInterval &floatExactOrIntervalMsg) {
    FloatExactOrInterval floatExactOrInterval;

    if (floatExactOrIntervalMsg.has_exact())
        floatExactOrInterval = floatExactOrIntervalMsg.exact();
    else if (floatExactOrIntervalMsg.has_interval())
        floatExactOrInterval = ProtobufReader::createFloatIntervalFromMessage(floatExactOrIntervalMsg.interval());

    return floatExactOrInterval;
}
