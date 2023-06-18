//
// Created by Yannick Ballnath.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/geometry/circle.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_dynamic.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_map.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_scenario.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/traffic_sign_element.pb.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <fstream>
#include <google/protobuf/message_lite.h>
#include <memory>
#include <tuple>
#include <vector>
#include <variant>

namespace ProtobufReader {

using IntegerInterval = std::pair<int, int>;
using FloatInterval = std::pair<double, double>;

using IntegerExactOrInterval = std::variant<int, IntegerInterval>;
using FloatExactOrInterval = std::variant<double, FloatInterval>;

using LaneletContainer = std::unordered_map<size_t, std::shared_ptr<Lanelet>>;
using TrafficSignContainer = std::unordered_map<size_t, std::shared_ptr<TrafficSign>>;
using TrafficLightContainer = std::unordered_map<size_t, std::shared_ptr<TrafficLight>>;
using IncomingGroupContainer = std::unordered_map<size_t, std::shared_ptr<IncomingGroup>>;
using OutgoingGroupContainer = std::unordered_map<size_t, std::shared_ptr<OutgoingGroup>>;

/**
 * Loads a CommonRoadDynamic message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_dynamic::CommonRoadDynamic loadDynamicProtobufMessage(const std::string &filePath);

/**
 * Loads a CommonRoadMap message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_map::CommonRoadMap loadMapProtobufMessage(const std::string &filePath);

/**
 * Loads a CommonRoadScenario message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_scenario::CommonRoadScenario loadScenarioProtobufMessage(const std::string &filePath);


/**
 * Initializes container of lanelets.
 *
 * @param laneletContainer Lanelet container
 * @param commonRoadDnamicMsg CommonRoad message
 */
void initLaneletContainer(LaneletContainer& laneletContainer, const commonroad_map::CommonRoadMap& commonRoadMapMsg);

/**
 * Initializes container of traffic signs.
 *
 * @param trafficSignContainer Traffic sign container
 * @param commonRoadMapMsg CommonRoad message
 */
void initTrafficSignContainer(TrafficSignContainer& trafficSignContainer, const commonroad_map::CommonRoadMap& commonRoadMapMsg);

/**
 * Initializes container of traffic lights.
 *
 * @param trafficLightContainer Traffic light container
 * @param commonRoadMapMsg CommonRoad message
 */
void initTrafficLightContainer(TrafficLightContainer& trafficLightContainer, const commonroad_map::CommonRoadMap& commonRoadMapMsg);

/**
 * Initializes container of incomingGroups.
 *
 * @param incomingGroupContainer Incoming container
 * @param intersectionMsg CommonRoad message
 */
void initIncomingGroupContainer(IncomingGroupContainer& incomingGroupContainer, const commonroad_map::Intersection& intersectionMsg);

/**
 * Initializes container of outgoingGroups.
 *
 * @param outgoingGroupContainer OutgoingGroup container
 * @param intersectionMsg CommonRoad message
 */
void initOutgoingGroupContainer(OutgoingGroupContainer& outgoingGroupContainer, const commonroad_map::Intersection& intersectionMsg);

/**
 * Returns lanelet from container of lanelets.
 *
 * @param laneletId Lanelet id
 * @param laneletContainer Lanelet container
 * @return Lanelet
 */
std::shared_ptr<Lanelet> getLaneletFromContainer(size_t laneletId, LaneletContainer& laneletContainer);

/**
 * Returns traffic sign from container of traffic signs.
 *
 * @param trafficSignId Traffic sign id
 * @param trafficSignContainer Traffic sign container
 * @return Traffic sign
 */
std::shared_ptr<TrafficSign> getTrafficSignFromContainer(size_t trafficSignId, TrafficSignContainer& trafficSignContainer);

/**
 * Returns traffic light from container of traffic lights.
 *
 * @param trafficLightId Traffic light id
 * @param trafficLightContainer Traffic light container
 * @return Traffic light
 */
std::shared_ptr<TrafficLight> getTrafficLightFromContainer(size_t trafficLightId, TrafficLightContainer& trafficLightContainer);

/**
 * Returns incomingGroups from container of IncomingGroups.
 *
 * @param incomingGroupId IncomingGroup id
 * @param incomingGroupContainer IncomingGroup container
 * @return Incoming
 */
std::shared_ptr<IncomingGroup> getIncomingGroupFromContainer(size_t incomingGroupId, IncomingGroupContainer& incomingGroupContainer);

/**
 * Returns outgoingGroups from container of OutgoingGroups.
 *
 * @param outgoingGroupId OutgoingGroup id
 * @param outgoingGroupContainer OutgoingGroup container
 * @return Incoming
 */
std::shared_ptr<OutgoingGroup> getIncomingGroupFromContainer(size_t outgoingGroupId, OutgoingGroupContainer& outgoingGroupContainer);

/**
 * Creates CR scenario from protobuf message "CommonRoad".
 *
 * @param commonRoadMsg Protobuf message
 * @return Scenario
 */
std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double> createCommonRoadFromMessage(const commonroad_dynamic::CommonRoadDynamic& commonRoadDynamicMsg,
                                                                                                                     const commonroad_map::CommonRoadMap& commonRoadMapMsg,
                                                                                                                     const commonroad_scenario::CommonRoadScenario& commonRoadScenarioMsg);

/**
 * Creates scenario information from protobuf message "ScenarioInformation".
 *
 * @param scenarioInformationMsg Protobuf message
 * @return Benchmark id and step size
 */
std::tuple<std::string, double> createScenarioInformationFromMessage(const commonroad_scenario::ScenarioMetaInformation& scenarioInfoMsg);

/**
 * Creates lanelet from protobuf message "Lanelet".
 *
 * @param laneletMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @return Lanelet
 */
std::shared_ptr<Lanelet> createLaneletFromMessage(const commonroad_map::Lanelet& laneletMsg, LaneletContainer& laneletContainer, TrafficSignContainer& trafficSignContainer, TrafficLightContainer& trafficLightContainer, const commonroad_map::CommonRoadMap& commonRoadMapMsg);

/**
 * Creates boundary from protobuf message "Bound".
 *
 * @param boundMsg Protobuf message
 * @return Vertices and line marking
 */
std::vector<vertex> createBoundFromMessage(const commonroad_map::Bound& boundMsg);

/**
 * Creates stop line from protobuf message "StopLine".
 *
 * @param stopLineMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @return Stop line
 */
std::shared_ptr<StopLine> createStopLineFromMessage(const commonroad_map::StopLine& stopLineMsg);

/**
 * Creates traffic sign from protobuf message "TrafficSign".
 *
 * @param trafficSignMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @return Traffic sign
 */
std::shared_ptr<TrafficSign> createTrafficSignFromMessage(const commonroad_map::TrafficSign& trafficSignMsg, TrafficSignContainer& trafficSignContainer);

/**
 * Creates traffic sign element from protobuf message "TrafficSignElement".
 *
 * @param trafficSignElementMsg Protobuf message
 * @return Traffic sign element
 */
std::shared_ptr<TrafficSignElement> createTrafficSignElementFromMessage(const commonroad_common::TrafficSignElement& trafficSignElementMsg);

/**
 * Creates traffic light from protobuf message "TrafficLight".
 *
 * @param trafficLightMsg Protobuf message
 * @param trafficLightContainer Traffic light container
 * @return Traffic light
 */
std::shared_ptr<TrafficLight> createTrafficLightFromMessage(const commonroad_map::TrafficLight& trafficLightMsg, TrafficLightContainer& trafficLightContainer);

/**
 * Creates traffic light cycle element from protobuf message "CycleElement".
 *
 * @param cycleElementMsg Protobuf message
 * @return Traffic light cycle element
 */
TrafficLightCycleElement createCycleElementFromMessage(const commonroad_dynamic::CycleElement& cycleElementMsg);

/**
 * Creates intersection from protobuf message "Intersection".
 *
 * @param intersectionMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @return Intersection
 */
std::shared_ptr<Intersection> createIntersectionFromMessage(const commonroad_map::Intersection& intersectionMsg, LaneletContainer& laneletContainer);

/**
 * Creates incomingGroup from protobuf message "incomingGroup".
 *
 * @param incomingGroupMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param incomingGroupContainer IncomingGroup container
 * @return Incoming
 */
std::shared_ptr<IncomingGroup> createIncomingGroupFromMessage(const commonroad_map::IncomingGroup& incomingGroupMsg, LaneletContainer& laneletContainer, IncomingGroupContainer& incomingGroupContainer);

/**
 * Creates outgoingGroup from protobuf message "outgoingGroup".
 *
 * @param outgoingGroupMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param outgoingGroupContainer IncomingGroup container
 * @return Incoming
 */
std::shared_ptr<OutgoingGroup> createOutgoingGroupFromMessage(const commonroad_map::OutgoingGroup& outgoingGroupMsg, LaneletContainer& laneletContainer, OutgoingGroupContainer& outgoingGroupContainer);


/**
 * Creates obstacle from protobuf message "StaticObstacle".
 *
 * @param staticObstacleMsg Protobuf message
 * @return Static obstacle
 */
std::shared_ptr<Obstacle> createStaticObstacleFromMessage(const commonroad_dynamic::StaticObstacle& staticObstacleMsg);

/**
 * Creates obstacle from protobuf message "DynamicObstacle".
 *
 * @param dynamicObstacleMsg Protobuf message
 * @return Dynamic obstacle
 */
std::shared_ptr<Obstacle> createDynamicObstacleFromMessage(const commonroad_dynamic::DynamicObstacle& dynamicObstacleMsg);

/**
 * Creates obstacle from protobuf message "EnvironmentObstacle".
 *
 * @param environmentObstacleMsg Protobuf message
 * @return Environment obstacle
 */
std::shared_ptr<Obstacle> createEnvironmentObstacleFromMessage(const commonroad_dynamic::EnvironmentObstacle& environmentObstacleMsg);

/**
 * Creates obstacle from protobuf message "PhantomObstacle".
 *
 * @param phantomObstacleMsg Protobuf message
 * @return Phantom obstacle
 */
std::shared_ptr<Obstacle> createPhantomObstacleFromMessage(const commonroad_dynamic::PhantomObstacle& phantomObstacleMsg);

/**
 * Creates sequence of states from protobuf message "TrajectoryPrediction".
 *
 * @param trajectoryPredictionMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>> createTrajectoryPredictionFromMessage(const commonroad_dynamic::TrajectoryPrediction& trajectoryPredictionMsg);

/**
 * Creates sequence of states from protobuf message "Trajectory".
 *
 * @param trajectoryMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>> createTrajectoryFromMessage(const commonroad_dynamic::Trajectory& trajectoryMsg);

/**
 * Creates state from protobuf message "State".
 *
 * @param stateMsg Protobuf message
 * @return State
 */
std::shared_ptr<State> createStateFromMessage(const State& stateMsg);

/**
 * Creates signal state from protobuf message "SignalState".
 * @param stateMsg Protobuf message
 * @return State
 */
std::shared_ptr<SignalState> createSignalStateFromMessage(const commonroad_common::SignalState &stateMsg);

/**
 * Creates vertex from protobuf message "Point".
 *
 * @param pointMsg Protobuf message
 * @return Vertex
 */
vertex createPointFromMessage(const commonroad_common::Point& pointMsg);

/**
 * Creates shape from protobuf message "Shape".
 *
 * @param shapeMsg Protobuf message
 * @return Shape
 */
std::unique_ptr<Shape> createShapeFromMessage(const commonroad_common::Shape& shapeMsg);

/**
 * Creates rectangle from protobuf message "Rectangle".
 *
 * @param rectangleMsg Protobuf message
 * @return Rectangle
 */
std::unique_ptr<Rectangle> createRectangleFromMessage(const commonroad_common::Rectangle& rectangleMsg);

/**
 * Creates circle from protobuf message "Circle".
 *
 * @param circleMsg Protobuf message
 * @return Circle
 */
std::unique_ptr<Circle> createCircleFromMessage(const commonroad_common::Circle& circleMsg);

/**
 * Creates integer interval from protobuf message "IntegerInterval".
 *
 * @param integerIntervalMsg Protobuf message
 * @return Integer interval
 */
IntegerInterval createIntegerIntervalFromMessage(const commonroad_common::IntegerInterval& integerIntervalMsg);

/**
 * Creates float interval from protobuf message "FloatInterval".
 *
 * @param floatIntervalMsg Protobuf message
 * @return Float interval
 */
FloatInterval createFloatIntervalFromMessage(const commonroad_common::FloatInterval& floatIntervalMsg);

/**
 * Creates either integer or integer interval from protobuf message "IntegerExactOrInterval".
 *
 * @param integerExactOrIntervalMsg Protobuf message
 * @return Integer or integer interval
 */
IntegerExactOrInterval createIntegerExactOrInterval(const commonroad_common::IntegerExactOrInterval& integerExactOrIntervalMsg);

/**
 * Creates either float or float interval from protobuf message "FloatExactOrInterval".
 *
 * @param floatExactOrIntervalMsg Protobuf message
 * @return Float or float interval
 */
FloatExactOrInterval createFloatExactOrInterval(const commonroad_common::FloatExactOrInterval& floatExactOrIntervalMsg);

}
