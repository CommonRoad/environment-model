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
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <omp.h>

class Lanelet;
class Lane;
class RoadNetwork;

/**
 * Class representing an obstacle.
 */
class Obstacle {
  public:
    /**
     * Default constructor without parameters for an obstacle.
     */
    Obstacle() = default;

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param obstacleId ID of obstacle.
     * @param isStatic Boolean indicating whether the obstacle is static or not.
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
     * @param route Planned route of obstacle.
     */
    Obstacle(size_t obstacleId, bool isStatic, std::shared_ptr<State> currentState, ObstacleType obstacleType,
             double vMax, double aMax, double aMaxLong, double aMinLong, double reactionTime,
             std::map<size_t, std::shared_ptr<State>> trajectoryPrediction, double length, double width,
             std::vector<vertex> route = {});

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
     * Setter for current state.
     *
     * @param currentState Current state of obstacle.
     */
    void setCurrentState(const std::shared_ptr<State> &currentState);

    /**
     * Setter for obstacle type.
     *
     * @param type Type of the obstacle.
     */
    void setObstacleType(ObstacleType type);

    /**
     * Setter for maximum velocity the obstacle can drive.
     *
     * @param vmax Maximum velocity [m/s]
     */
    void setVmax(double vmax);

    /**
     * Setter for maximum acceleration the obstacle can have.
     *
     * @param amax Maximum acceleration [m/s^2]
     */
    void setAmax(double amax);

    /**
     * Setter for maximum acceleration the obstacle can have in longitudinal direction.
     *
     * @param amax Maximum acceleration [m/s^2]
     */
    void setAmaxLong(double amax);

    /**
     * Setter for minimum acceleration the obstacle can have in the longitudinal direction.
     *
     * @param amin Minimum acceleration [m/s^2]
     */
    void setAminLong(double amin);

    /**
     * Setter for reaction time.
     *
     * @param tReact Reaction time of the obstacle [s].
     */
    void setReactionTime(double tReact);

    /**
     * Setter for trajectory prediction.
     *
     * @param trajPrediction Map matching time step to state.
     */
    void setTrajectoryPrediction(const std::map<size_t, std::shared_ptr<State>> &trajPrediction);

    /**
     * Setter for obstacle shape.
     *
     * @param length Length of the obstacle [m].
     * @param width Width of the obstacle [m].
     */
    void setRectangleShape(double length, double width);

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
     * Setter for route.
     *
     * @param newRoute New route of obstacle.
     */
    void setRoute(const std::vector<vertex> &newRoute);

    /**
     * Getter for obstacle ID.
     *
     * @return Obstacle ID.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for isStatic.
     *
     * @return Boolean indicating whether the obstacle is static or not.
     */
    [[nodiscard]] bool getIsStatic() const;

    /**
     * Getter for current state.
     *
     * @return Pointer to state object.
     */
    [[nodiscard]] const std::shared_ptr<State> &getCurrentState() const;

    /**
     * Getter for obstacle type.
     *
     * @return Type of the obstacle.
     */
    [[nodiscard]] ObstacleType getObstacleType() const;

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
    [[nodiscard]] double getReactionTime() const;

    /**
     * Getter for reference lane.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    [[nodiscard]] std::shared_ptr<Lane> getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                         size_t timeStep);

    /**
     * Getter for trajectory prediction.
     *
     * @return Map matching time step to state.
     */
    [[nodiscard]] std::map<size_t, std::shared_ptr<State>> getTrajectoryPrediction() const;

    /**
     * Getter for polygon shape of obstacle at given time step.
     *
     * @param timeStep Time step of interest.
     * @return Boost polygon.
     */
    [[nodiscard]] polygon_type getOccupancyPolygonShape(size_t timeStep);

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
    getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Provides state given a time step. The time step can belong to the current state, history, or prediction.
     *
     * @param timeStep Time step of interest.
     * @return Pointer to state object.
     */
    [[nodiscard]] std::shared_ptr<State> getStateByTimeStep(size_t timeStep) const;

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
    double frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes) based on a given reference
     * lane.
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane which should be used.
     * @return Longitudinal position of obstacle front
     */
    double frontS(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes)
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle rear
     */
    double rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Calculates right d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @return right d-coordinate [m]
     */
    double rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Calculates left d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @return left d-coordinate [m]
     */
    double leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Calculates left lateral position of obstacle based on given reference lane
     *
     * @param timeStep time step to consider
     * @param refLane Pointer to reference lane which should be used.
     * @return lateral position [m]
     */
    double leftD(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Calculates right lateral position of obstacle based on given reference lane
     *
     * @param timeStep time step to consider
     * @param refLane Pointer to reference lane which should be used.
     * @return lateral position [m]
     */
    double rightD(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes) based on given reference lane.
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane which should be used.
     * @return Longitudinal position.
     */
    double rearS(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and provided reference lane.
     *
     * @param timeStep Time Step of interest.
     * @param refLane Pointer to reference lane.
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param timeStep Time step of interest.
     * @param refLane Pointer to reference lane.
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane);

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and assigned lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                   size_t timeStep); // Todo create test case

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and provided reference lane
     *
     * @param timeStep time step of interest
     * @param refLane Reference lane based on which curvilinear orientation should be computed.
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(size_t timeStep,
                                                   const std::shared_ptr<Lane> &refLane); // Todo create test case

    /**
     * Sets the lanes from the road network the obstacle occupies at a certain time step
     *
     * @param timeStep time step of interest
     * @return list of pointers to occupied lanes
     */
    void setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep); // TODO create test case

    /**
     * Computes occupied lanes at a time step.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     */
    void setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep); // TODO create test case

    /**
     * Checks whether occupied lanes are already computed for a time step.
     *
     * @param timeStep Time step of interest.
     */
    bool existsOccupiedLanes(size_t timeStep); // TODO create test case

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
                                                        size_t timeStep); // TODO create test case

    /**
     * Computes driving paths at time step. Driving path is defined by all lanes completely adjacent to reference lane.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to lanes which are part of driving path.
     */
    std::vector<std::shared_ptr<Lane>> getDrivingPathLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                           size_t timeStep); // TODO create test case

    /**
     * Getter for route.
     *
     * @return Route as list of vertices.
     */
    const std::vector<vertex> &getRoute() const;

    /**
     * Converts the x- and y-coordinate into the Curvilinear domain given own reference lane.
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep Time step for which the coordinates should me transformed.
     */
    void convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Checks whether time steps exists in trajectory prediction or current state.
     *
     * @param timeStep Time step of interest.
     * @return Boolean indicating whether time step exists.
     */
    bool timeStepExists(size_t timeStep);

    /**
     * Interpolates acceleration based on velocity.
     *
     * @param timeStep Time step for which acceleration should be interpolated.
     * @param timeStepSize Time step size [s].
     */
    void interpolateAcceleration(size_t timeStep, double timeStepSize);

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
    void convertPointToCurvilinear(size_t timeStep, const std::shared_ptr<Lane> &refLane);

  private:
    size_t obstacleId;                                //**< unique ID of obstacle */
    bool isStatic{false};                             //**< true if Obstacle is static */
    std::shared_ptr<State> currentState;              //**< pointer to current state of obstacle */
    ObstacleType obstacleType{ObstacleType::unknown}; //**< CommonRoad obstacle type */
    double vMax{50.0};                                //**< maximum velocity of obstacle in m/s */
    double aMax{3.0};                                 //**< maximum absolute acceleration of obstacle in [m/s^2] */
    double aMaxLong{3.0};                             //**< maximal longitudinal acceleration of obstacle in [m/s^2] */
    double aMinLong{-10.0};                           //**< minimal longitudinal acceleration of obstacle in [m/s^2] */
    double reactionTime{0.3};                         //**< reaction time of obstacle in [s] */
    std::map<size_t, std::shared_ptr<State>> trajectoryPrediction{}; //**< trajectory prediction of the obstacle */
    std::map<size_t, std::shared_ptr<State>> history{};              //**< previous states of the obstacle */
    Rectangle geoShape; // TODO make general                                          //**< shape of the obstacle */
    std::map<size_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLanelets; //**< map of time steps to lanelets occupied by the obstacle */
    std::map<size_t, std::shared_ptr<Lane>>
        referenceLane; //**< lane which is used as reference for curvilinear projection */
    std::map<size_t, std::vector<std::shared_ptr<Lane>>>
        occupiedLanes; //**< map of time steps to lanes occupied by the obstacle */
    std::map<size_t, std::map<std::set<size_t>, std::array<double, 3>>>
        convertedPositions; //**< map of time steps to lanelet ID set to curvilinear positions */
    std::map<size_t, polygon_type> shapeAtTimeStep; //**< occupied polygon shape at time steps */
    std::vector<vertex> route;                      //**< planned route of the obstacle */
    const double laneOrientationThreshold{0.7};     //**< orientation threshold for assigning lanes */
    const double laneOrientationThresholdInitial{
        1.58}; //**< orientation threshold for assigning lanes at initial time step, should be larger than other
               // threshold since initial time step has special evaluation */
    const double fieldOfViewRear{250.0};  //**< length of field of view provided by front sensors */
    const double fieldOfViewFront{250.0}; //**< length of field of view provided by rear sensors */

    /**
     * Private setter for occupied lanelets at a time steps within a road network.
     * Used to define critical section around it.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @return List of pointers to occupied lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                     size_t timeStep);

    /**
     * Private setter for polygon shape of obstacle at given time step.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @return Boost polygon.
     */
    polygon_type setOccupancyPolygonShape(size_t timeStep);

    /**
     * Private setter for reference lane.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    std::shared_ptr<Lane> setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);
};
