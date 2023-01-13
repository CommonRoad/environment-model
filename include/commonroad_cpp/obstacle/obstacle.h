//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

#include "state.h"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/geometry/types.h>
#include <commonroad_cpp/roadNetwork/types.h>
#include <commonroad_cpp/roadNetwork/road_network_config.h>

#include <boost/container_hash/hash.hpp>

#include "actuator_parameters.h"
#include "sensor_parameters.h"
#include "signal_state.h"

#include <tsl/robin_map.h>

class Lanelet;
class Lane;
class RoadNetwork;

/**
 * Class representing an obstacle.
 */
class Obstacle {
  public:
    template <typename Value>
    using time_step_map_t = tsl::robin_map<time_step_t, Value>;
    //** type of history/trajectory prediction maps for physical states */
    using state_map_t = time_step_map_t<std::shared_ptr<State>>;
    //** type of history/trajectory prediction maps for signal states*/
    using signal_state_map_t = time_step_map_t<std::shared_ptr<SignalState>>;

    /**
     * Default constructor without parameters for an obstacle.
     */
    Obstacle() = default;

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param obstacleId ID of obstacle.
     * @param obstacleRole CommonRoad obstacle role, e.g, dynamic, static, environment, etc.
     * @param currentState Pointer to current state of obstacle.
     * @param obstacleType Type of the obstacle.
     * @param vMax Maximum velocity the obstacle can drive [m/s].
     * @param aMax Maximum acceleration the obstacle can have [m/s^2].
     * @param aMaxLong Maximum acceleration the obstacle can have in longitudinal direction [m/s^2].
     * @param aMinLong Minimum acceleration the obstacle can have in the longitudinal direction [m/s^2].
     * @param reactionTime Reaction time of the obstacle [s].
     * @param trajectoryPrediction Map matching time step to state.
     * @param length Length of the obstacle [m].
     * @param width Width of the obstacle [m].
     * @param fov Field of view.
     */
    Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState, ObstacleType obstacleType,
             double vMax, double aMax, double aMaxLong, double aMinLong, std::optional<double> reactionTime,
             state_map_t trajectoryPrediction, double length, double width, const std::vector<vertex> &fov = {});

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param obstacleId ID of obstacle.
     * @param isStatic Boolean indicating whether the obstacle is static or not.
     * @param currentState Pointer to current state of obstacle.
     * @param obstacleType Type of the obstacle.
     * @param actuatorParameters Kinematic parameters of the obstacle.
     * @param trajectoryPrediction Map matching time step to state.
     * @param shape Obstacle shape. (only rectangles are currently supported!)
     * @param fov Field of view.
     */
    Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState, ObstacleType obstacleType,
             ActuatorParameters actuatorParameters, SensorParameters sensorParameters, state_map_t trajectoryPrediction,
             std::unique_ptr<Shape> shape, const std::vector<vertex> &fov);

    /**
     * Setter for ID of obstacle.
     *
     * @param obstacleId ID of obstacle.
     */
    void setId(size_t obstacleId);

    /**
     * Setter for isStatic.
     *
     * @param staticObstacle Boolean indicating whether the obstacle is static or not.
     */
    void setIsStatic(bool staticObstacle);

    /**
     * Setter for obstacle role.
     *
     * @param type Role of obstacle
     */
     void setObstacleRole(ObstacleRole type);

    /**
     * Setter for current state.
     *
     * @param currentState Current state of obstacle.
     */
    void setCurrentState(const std::shared_ptr<State> &currentState);

    /**
     * Setter for current signal state.
     *
     * @param state Current signal state of obstacle.
     */
    void setCurrentSignalState(const std::shared_ptr<SignalState> &state);

    /**
     * Setter for obstacle type.
     *
     * @param type Type of the obstacle.
     */
    void setObstacleType(ObstacleType type);

    /**
     * Setter for actuator parameters.
     *
     * @param actuatorParameters Actuator parameters
     */
    void setActuatorParameters(ActuatorParameters actuatorParameters);

    /**
     * Setter for sensor parameters.
     *
     * @param sensorParameters Sensor parameters
     */
    void setSensorParameters(SensorParameters sensorParameters);

    /**
     * Setter for trajectory prediction.
     *
     * @param trajPrediction Map matching time step to state.
     */
    void setTrajectoryPrediction(const state_map_t &trajPrediction);

    /**
     * Setter for obstacle shape of type rectangle.
     *
     * @param length Length of the obstacle [m].
     * @param width Width of the obstacle [m].
     */
    void setRectangleShape(double length, double width);

    /**
     * Setter for obstacle of shape type circle.
     *
     * @param radius Radius of obstacle [m].
     * @param center Center of circle as vertex.
     */
    void setCircleShape(double radius, vertex center = {});

    /**
     * Setter for obstacle shape
     * @param geoShape Shape
     */
    void setGeoShape(std::unique_ptr<Shape> shape);

    /**
     * Appends a state to the trajectory prediction.
     *
     * @param state Pointer to state object.
     */
    void appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state);

    /**
     * Appends a state to the history.
     *
     * @param state Pointer to state object.
     */
    void appendStateToHistory(const std::shared_ptr<State> &state);

    /**
     * Appends a signal state to the signal series.
     *
     * @param state Pointer to signal state object.
     */
    void appendSignalStateToSeries(const std::shared_ptr<SignalState> &state);

    /**
     * Appends a signal state to the history.
     *
     * @param state Pointer to signal state object.
     */
    void appendSignalStateToHistory(const std::shared_ptr<SignalState> &state);

    /**
     * Getter for obstacle ID.
     *
     * @return Obstacle ID.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for obstacle role.
     *
     * @return Role of obstacle
     */
    [[nodiscard]] ObstacleRole getObstacleRole() const;

    /**
     * Getter for current state.
     *
     * @return Pointer to state object.
     */
    [[nodiscard]] const std::shared_ptr<State> &getCurrentState() const;

    /**
     * Getter current signal state.
     *
     * @return Pointer to signal state object.
     */
    [[nodiscard]] const std::shared_ptr<SignalState> &getCurrentSignalState() const;

    /**
     * Getter for obstacle type.
     *
     * @return Type of the obstacle.
     */
    [[nodiscard]] ObstacleType getObstacleType() const;

    /**
     * Getter for actuator parameters.
     *
     * @param actuatorParameters Actuator parameters
     */
    ActuatorParameters getActuatorParameters() const;

    /**
     * Getter for sensor parameters.
     *
     * @param sensorParameters Sensor parameters
     */
    SensorParameters getSensorParameters() const;

    /**
     * Getter for maximum velocity the vehicle can drive.
     *
     * @return Maximum velocity [m/s].
     */
    [[nodiscard]] double getVmax() const;

    /**
     * Getter for maximum acceleration.
     *
     * @return Maximum acceleration [m/s^2].
     */
    [[nodiscard]] double getAmax() const;

    /**
     * Getter for maximum acceleration in longitudinal direction.
     *
     * @return Maximum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAmaxLong() const;

    /**
     * Getter for minimum acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAminLong() const;

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] std::optional<double> getReactionTime() const;

    /**
     * Getter for reference lane.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    [[nodiscard]] std::shared_ptr<Lane> getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                         time_step_t timeStep);

    /**
     * Getter for trajectory prediction.
     *
     * @return Map matching time step to state.
     */
    [[nodiscard]] state_map_t getTrajectoryPrediction() const;

    /**
     * Getter for polygon shape of obstacle at given time step.
     *
     * @param timeStep Time step of interest.
     * @return Boost polygon.
     */
    [[nodiscard]] polygon_type getOccupancyPolygonShape(time_step_t timeStep);

    /**
     * Getter for obstacle shape.
     *
     * @return Shape object.
     */
    [[nodiscard]] Shape &getGeoShape();

    /**
     * Getter for occupied lanelets at a time steps within a road network.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @return List of pointers to occupied lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Provides state given a time step. The time step can belong to the current state, history, or prediction.
     *
     * @param timeStep Time step of interest.
     * @return Pointer to state object.
     */
    [[nodiscard]] std::shared_ptr<State> getStateByTimeStep(time_step_t timeStep) const;

    /**
     * Provides signalState given a time step. The time step can belong to the current state, history, or prediction.
     *
     * @param timeStep Time step of interest.
     * @return Pointer to signalState object.
     */
    [[nodiscard]] std::shared_ptr<SignalState> getSignalStateByTimeStep(time_step_t timeStep) const;

    /**
     * Returns the length of the trajectory prediction.
     *
     * @return Length of trajectory prediction.
     */
    [[nodiscard]] size_t getTrajectoryLength() const;

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes)
     *
     * @param timeStep time step of interest
     * @param roadNetwork Pointer to road network
     * @return longitudinal position of obstacle front
     */
    double frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes) based on a given reference
     * lane.
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane which should be used.
     * @return Longitudinal position of obstacle front
     */
    double frontS(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes)
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle rear
     */
    double rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Calculates right d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @return right d-coordinate [m]
     */
    double rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Calculates left d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @return left d-coordinate [m]
     */
    double leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Calculates left lateral position of obstacle based on given reference lane
     *
     * @param timeStep time step to consider
     * @param refLane Pointer to reference lane which should be used.
     * @return lateral position [m]
     */
    double leftD(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Calculates right lateral position of obstacle based on given reference lane
     *
     * @param timeStep time step to consider
     * @param refLane Pointer to reference lane which should be used.
     * @return lateral position [m]
     */
    double rightD(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes) based on given reference lane.
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane which should be used.
     * @return Longitudinal position.
     */
    double rearS(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and provided reference lane.
     *
     * @param timeStep Time Step of interest.
     * @param refLane Pointer to reference lane.
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane.
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and assigned lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                   time_step_t timeStep); // Todo create test case

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and provided reference lane
     *
     * @param timeStep time step of interest
     * @param refLane Reference lane based on which curvilinear orientation should be computed.
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(time_step_t timeStep,
                                                   const std::shared_ptr<Lane> &refLane); // Todo create test case

    /**
     * Sets the lanes from the road network the obstacle occupies at a certain time step
     *
     * @param timeStep time step of interest
     * @return list of pointers to occupied lanes
     */
    void setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes,
                          time_step_t timeStep); // TODO create test case

    /**
     * Computes occupied lanes at a time step.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     */
    void setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                          time_step_t timeStep); // TODO create test case

    /**
     * Extracts first time step of trajectory
     *
     * @return first time step of trajectory
     */
    [[nodiscard]] size_t getFirstTrajectoryTimeStep(); // TODO create test case

    /**
     * Extracts last time step of trajectory
     *
     * @return last time step of trajectory
     */
    [[nodiscard]] size_t getLastTrajectoryTimeStep() const; // TODO create test case

    /**
     * Getter for occupied lanes at a time step. Computes occupied lanes if not happened already.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to occupied lanes.
     */
    std::vector<std::shared_ptr<Lane>> getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        time_step_t timeStep); // TODO create test case

    /**
     * Computes driving paths at time step. Driving path is defined by all lanes completely adjacent to reference lane.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to lanes which are part of driving path.
     */
    std::vector<std::shared_ptr<Lane>> getDrivingPathLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                           time_step_t timeStep); // TODO create test case

    /**
     * Converts the x- and y-coordinate into the Curvilinear domain given own reference lane.
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep Time step for which the coordinates should me transformed.
     */
    void convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Checks whether time steps exists in trajectory prediction or current state.
     *
     * @param timeStep Time step of interest.
     * @return Boolean indicating whether time step exists.
     */
    bool timeStepExists(time_step_t timeStep);

    /**
     * Interpolates acceleration based on velocity.
     *
     * @param timeStep Time step for which acceleration should be interpolated.
     * @param timeStepSize Time step size [s].
     */
    void interpolateAcceleration(time_step_t timeStep, double timeStepSize);

    /**
     * Getter for all prediction time steps.
     *
     * @return List of time steps of prediction.
     */
    std::vector<size_t> getPredictionTimeSteps();

    /**
     * Getter for all history time steps.
     *
     * @return List of time steps of prediction.
     */
    std::vector<size_t> getHistoryTimeSteps();

    /**
     * Getter for list of time steps containing current time step and prediction time steps.
     *
     * @return List of time steps.
     */
    std::vector<size_t> getTimeSteps();

    /**
     * Computes occupied lanes for each time step of obstacle and sets reference lane.
     *
     * @param roadNetwork Pointer to road network.
     * @param considerHistory Boolean indicating whether history should be considered for computation
     */
    void computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, bool considerHistory = false);

    /**
     * Converts all states to curvilinear representation.
     *
     * @param roadNetwork Pointer to road network
     */
    void setCurvilinearStates(const std::shared_ptr<RoadNetwork> &roadNetwork);

    /**
     * Converts position at a given time step to curvilinear coordinate system given a reference lane.
     * Point is stored locally in variable convertedPositions.
     * @param timeStep time step of interest
     * @param refLane Reference lane which should be used.
     */
    void convertPointToCurvilinear(time_step_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Getter for field of view area.
     *
     * @return Field of view area as polygon.
     */
    const polygon_type &getFov() const;

    /**
     * Setter for field of view area.
     *
     * @param fovVertices Vertices representing polygon.
     */
    void setFov(const std::vector<vertex> &fovVertices);

    /**
     * Getter for signal series.
     *
     * @return Map representing signal series.
     */
    const signal_state_map_t &getSignalSeries() const;

    /**
     * Getter for signal series history.
     *
     * @return Map representing signal series history.
     */
    const signal_state_map_t &getSignalSeriesHistory() const;

    /**
     * Checks whether obstacle is static.
     *
     * @return Boolean indicating whether obstacle is static.
     */
    bool isStatic() const;

    /**
     * Getter for FieldOfViewRear.
     *
     * @return Distance of FieldOfViewRear.
     */
    double getFieldOfViewRearDistance() const;

    /**
     * Getter for FieldOfViewFront
     *
     * @return Distance of FieldOfViewFront.
     */
    double getFieldOfViewFrontDistance() const;

    /**
     * Setter for fieldOfViewRear.
     *
     * @param distance fieldOfViewRear Distance.
     */
    void setFieldOfViewRearDistance(double distance);

    /**
     * Setter for fieldOfViewFront.
     *
     * @param distance fieldOfViewFront Distance.
     */
    void setFieldOfViewFrontDistance(double distance);

  private:
    size_t obstacleId;                                //**< unique ID of obstacle */
    ObstacleRole obstacleRole{ObstacleRole::DYNAMIC}; //**< CommonRoad obstacle role */
    std::shared_ptr<State> currentState;              //**< pointer to current state of obstacle */
    std::shared_ptr<SignalState> currentSignalState;  //**< pointer to current signal state of obstacle */
    ObstacleType obstacleType{ObstacleType::unknown}; //**< CommonRoad obstacle type */

    std::optional<ActuatorParameters> actuatorParameters;
    std::optional<SensorParameters> sensorParameters;

    state_map_t trajectoryPrediction{}; //**< trajectory prediction of the obstacle */
    signal_state_map_t signalSeries{};         //**< signal series of the obstacle */
    state_map_t trajectoryHistory{};              //**< previous states of the obstacle */
    signal_state_map_t signalSeriesHistory{};         //**< previous signal states of the obstacle */

    std::unique_ptr<Shape>
        geoShape; // TODO make general                                          //**< shape of the obstacle */

    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLanelets; //**< map of time steps to lanelets occupied by the obstacle */

    mutable time_step_map_t<std::vector<std::shared_ptr<Lane>>>
        occupiedLanesDrivingDir; //**< map of time steps to lanelets occupied by the obstacle */

    mutable time_step_map_t<std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsDrivingDir; //**< map of time steps to lanelets in driving direction occupied by the obstacle */

    mutable time_step_map_t<std::shared_ptr<Lane>>
        referenceLane; //**< lane which is used as reference for curvilinear projection */

    mutable time_step_map_t<std::vector<std::shared_ptr<Lane>>>
        occupiedLanes; //**< map of time steps to lanes occupied by the obstacle */

    using curvilinear_position_t = std::array<double, 3>; //**< curvilinear position of an obstacle */
    using curvilinear_position_map_t =
        tsl::robin_pg_map<std::shared_ptr<Lane>, curvilinear_position_t>; //**< map from lanelet ID set to curvilinear positions */
    mutable time_step_map_t<curvilinear_position_map_t>
        convertedPositions; //**< map of time steps to lanelet ID set to curvilinear positions */

    curvilinear_position_map_t::const_iterator find_or_convert(size_t timeStep, const std::shared_ptr<Lane> &refLane) const;

    mutable time_step_map_t<polygon_type> shapeAtTimeStep; //**< occupied polygon shape at time steps */

    double fieldOfViewRear{250.0};  //**< length of field of view provided by front sensors */
    double fieldOfViewFront{250.0}; //**< length of field of view provided by rear sensors */
    polygon_type fov;               //**< fov of vehicle captured by sensors */

    /**
     * Private setter for occupied lanelets at a time steps within a road network.
     * Used to define critical section around it.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @return List of pointers to occupied lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                     time_step_t timeStep);

    /**
     * Sets the occupied lanelets in driving direction for a time step.
     *
     * @param roadNetwork Road network.
     * @param timeStep Time step of interest.
     * @return List of occupied lanelets in driving direction.
     */
    std::vector<std::shared_ptr<Lanelet>>
    setOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Private setter for polygon shape of obstacle at given time step.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @return Boost polygon.
     */
    polygon_type setOccupancyPolygonShape(time_step_t timeStep);

    /**
     * Private setter for reference lane.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    std::shared_ptr<Lane> setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes main reference path of obstacle at given time step.
     *
     * @param roadNetwork Road network.
     * @param timeStep Time step of interest.
     * @param lane Relevant lanes used for computing reference path.
     *
     * @return Lane which is used as reference path.
     */
    std::vector<std::shared_ptr<Lane>> computeMainRef(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                                      const std::vector<std::shared_ptr<Lane>> &lane);
};
