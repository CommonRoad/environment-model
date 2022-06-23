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
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses/commonroad.pb.h>
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
using IncomingContainer = std::unordered_map<size_t, std::shared_ptr<Incoming>>;

/**
 * Loads a CommonRoad message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad::CommonRoad loadProtobufMessage(const std::string &filePath);

/**
 * Initializes container of lanelets.
 *
 * @param laneletContainer Lanelet container
 * @param commonRoadMsg CommonRoad message
 */
void initLaneletContainer(LaneletContainer& laneletContainer, const commonroad::CommonRoad& commonRoadMsg);

/**
 * Initializes container of traffic signs.
 *
 * @param trafficSignContainer Traffic sign container
 * @param commonRoadMsg CommonRoad message
 */
void initTrafficSignContainer(TrafficSignContainer& trafficSignContainer, const commonroad::CommonRoad& commonRoadMsg);

/**
 * Initializes container of traffic lights.
 *
 * @param trafficLightContainer Traffic light container
 * @param commonRoadMsg CommonRoad message
 */
void initTrafficLightContainer(TrafficLightContainer& trafficLightContainer, const commonroad::CommonRoad& commonRoadMsg);

/**
 * Initializes container of incomings.
 *
 * @param incomingContainer Incoming container
 * @param intersectionMsg CommonRoad message
 */
void initIncomingContainer(IncomingContainer& incomingContainer, const commonroad::Intersection& intersectionMsg);

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
 * Returns incoming from container of incomings.
 *
 * @param incomingId Incoming id
 * @param incomingContainer Incoming container
 * @return Incoming
 */
std::shared_ptr<Incoming> getIncomingFromContainer(size_t incomingId, IncomingContainer& incomingContainer);

/**
 * Creates CR scenario from protobuf message "CommonRoad".
 *
 * @param commonRoadMsg Protobuf message
 * @return Scenario
 */
std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double> createCommonRoadFromMessage(const commonroad::CommonRoad& commonRoadMsg);

/**
 * Creates scenario information from protobuf message "ScenarioInformation".
 *
 * @param scenarioInformationMsg Protobuf message
 * @return Benchmark id and step size
 */
std::tuple<std::string, double> createScenarioInformationFromMessage(const commonroad::ScenarioInformation& scenarioInformationMsg);

/**
 * Creates lanelet from protobuf message "Lanelet".
 *
 * @param laneletMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @return Lanelet
 */
std::shared_ptr<Lanelet> createLaneletFromMessage(const commonroad::Lanelet& laneletMsg, LaneletContainer& laneletContainer, TrafficSignContainer& trafficSignContainer, TrafficLightContainer& trafficLightContainer);

/**
 * Creates boundary from protobuf message "Bound".
 *
 * @param boundMsg Protobuf message
 * @return Vertices and line marking
 */
std::tuple<std::vector<vertex>, std::unique_ptr<LineMarking>> createBoundFromMessage(const commonroad::Bound& boundMsg);

/**
 * Creates stop line from protobuf message "StopLine".
 *
 * @param stopLineMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @return Stop line
 */
std::shared_ptr<StopLine> createStopLineFromMessage(const commonroad::StopLine& stopLineMsg, TrafficSignContainer& trafficSignContainer, TrafficLightContainer& trafficLightContainer);

/**
 * Creates traffic sign from protobuf message "TrafficSign".
 *
 * @param trafficSignMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @return Traffic sign
 */
std::shared_ptr<TrafficSign> createTrafficSignFromMessage(const commonroad::TrafficSign& trafficSignMsg, TrafficSignContainer& trafficSignContainer);

/**
 * Creates traffic sign element from protobuf message "TrafficSignElement".
 *
 * @param trafficSignElementMsg Protobuf message
 * @return Traffic sign element
 */
std::shared_ptr<TrafficSignElement> createTrafficSignElementFromMessage(const commonroad::TrafficSignElement& trafficSignElementMsg);

/**
 * Creates traffic light from protobuf message "TrafficLight".
 *
 * @param trafficLightMsg Protobuf message
 * @param trafficLightContainer Traffic light container
 * @return Traffic light
 */
std::shared_ptr<TrafficLight> createTrafficLightFromMessage(const commonroad::TrafficLight& trafficLightMsg, TrafficLightContainer& trafficLightContainer);

/**
 * Creates traffic light cycle element from protobuf message "CycleElement".
 *
 * @param cycleElementMsg Protobuf message
 * @return Traffic light cycle element
 */
TrafficLightCycleElement createCycleElementFromMessage(const commonroad::CycleElement& cycleElementMsg);

/**
 * Creates intersection from protobuf message "Intersection".
 *
 * @param intersectionMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @return Intersection
 */
std::shared_ptr<Intersection> createIntersectionFromMessage(const commonroad::Intersection& intersectionMsg, LaneletContainer& laneletContainer);

/**
 * Creates incoming from protobuf message "Incoming".
 *
 * @param incomingMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param incomingContainer Incoming container
 * @return Incoming
 */
std::shared_ptr<Incoming> createIncomingFromMessage(const commonroad::Incoming& incomingMsg, LaneletContainer& laneletContainer, IncomingContainer& incomingContainer);

/**
 * Creates obstacle from protobuf message "StaticObstacle".
 *
 * @param staticObstacleMsg Protobuf message
 * @return Static obstacle
 */
std::shared_ptr<Obstacle> createStaticObstacleFromMessage(const commonroad::StaticObstacle& staticObstacleMsg);

/**
 * Creates obstacle from protobuf message "DynamicObstacle".
 *
 * @param dynamicObstacleMsg Protobuf message
 * @return Dynamic obstacle
 */
std::shared_ptr<Obstacle> createDynamicObstacleFromMessage(const commonroad::DynamicObstacle& dynamicObstacleMsg);

/**
 * Creates obstacle from protobuf message "EnvironmentObstacle".
 *
 * @param environmentObstacleMsg Protobuf message
 * @return Environment obstacle
 */
std::shared_ptr<Obstacle> createEnvironmentObstacleFromMessage(const commonroad::EnvironmentObstacle& environmentObstacleMsg);

/**
 * Creates obstacle from protobuf message "PhantomObstacle".
 *
 * @param phantomObstacleMsg Protobuf message
 * @return Phantom obstacle
 */
std::shared_ptr<Obstacle> createPhantomObstacleFromMessage(const commonroad::PhantomObstacle& phantomObstacleMsg);

/**
 * Creates sequence of states from protobuf message "TrajectoryPrediction".
 *
 * @param trajectoryPredictionMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>> createTrajectoryPredictionFromMessage(const commonroad::TrajectoryPrediction& trajectoryPredictionMsg);

/**
 * Creates sequence of states from protobuf message "Trajectory".
 *
 * @param trajectoryMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>> createTrajectoryFromMessage(const commonroad::Trajectory& trajectoryMsg);

/**
 * Creates state from protobuf message "State".
 *
 * @param stateMsg Protobuf message
 * @return State
 */
std::shared_ptr<State> createStateFromMessage(const commonroad::State& stateMsg);

/**
 * Creates vertex from protobuf message "Point".
 *
 * @param pointMsg Protobuf message
 * @return Vertex
 */
vertex createPointFromMessage(const commonroad::Point& pointMsg);

/**
 * Creates shape from protobuf message "Shape".
 *
 * @param shapeMsg Protobuf message
 * @return Shape
 */
std::unique_ptr<Shape> createShapeFromMessage(const commonroad::Shape& shapeMsg);

/**
 * Creates rectangle from protobuf message "Rectangle".
 *
 * @param rectangleMsg Protobuf message
 * @return Rectangle
 */
std::unique_ptr<Rectangle> createRectangleFromMessage(const commonroad::Rectangle& rectangleMsg);

/**
 * Creates circle from protobuf message "Circle".
 *
 * @param circleMsg Protobuf message
 * @return Circle
 */
std::unique_ptr<Circle> createCircleFromMessage(const commonroad::Circle& circleMsg);

/**
 * Creates integer interval from protobuf message "IntegerInterval".
 *
 * @param integerIntervalMsg Protobuf message
 * @return Integer interval
 */
IntegerInterval createIntegerIntervalFromMessage(const commonroad::IntegerInterval& integerIntervalMsg);

/**
 * Creates float interval from protobuf message "FloatInterval".
 *
 * @param floatIntervalMsg Protobuf message
 * @return Float interval
 */
FloatInterval createFloatIntervalFromMessage(const commonroad::FloatInterval& floatIntervalMsg);

/**
 * Creates either integer or integer interval from protobuf message "IntegerExactOrInterval".
 *
 * @param integerExactOrIntervalMsg Protobuf message
 * @return Integer or integer interval
 */
IntegerExactOrInterval createIntegerExactOrInterval(const commonroad::IntegerExactOrInterval& integerExactOrIntervalMsg);

/**
 * Creates either float or float interval from protobuf message "FloatExactOrInterval".
 *
 * @param floatExactOrIntervalMsg Protobuf message
 * @return Float or float interval
 */
FloatExactOrInterval createFloatExactOrInterval(const commonroad::FloatExactOrInterval& floatExactOrIntervalMsg);

}
