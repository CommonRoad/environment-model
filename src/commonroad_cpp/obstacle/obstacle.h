//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

// #include "commonroad_cpp/auxiliaryDefs/structs.h"
// #include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
// #include "commonroad_cpp/geometry/rectangle.h"
// #include "commonroad_cpp/geometry/shape.h"
// #include "commonroad_cpp/roadNetwork/lanelet/lane.h"
// #include "commonroad_cpp/roadNetwork/road_network.h"

#include <cstddef>
#include <vector>
#include <memory>
#include <map>

#include "state.h"

// #include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/geometry/rectangle.h>

/*
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
*/
using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type = boost::geometry::model::polygon<point_type>;

class Lanelet;
class Lane;
class RoadNetwork;

/**
 * Class representing an obstacle.
 */
class Obstacle {
  public:
    /**
     * Default Constructor without parameters for an obstacle.
     */
    Obstacle() = default;

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param id ID of obstacle.
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
     */
    Obstacle(size_t id, bool isStatic, std::shared_ptr<State> currentState, ObstacleType obstacleType, double vMax,
             double aMax, double aMaxLong, double aMinLong, double reactionTime,
             std::map<size_t, std::shared_ptr<State>> trajectoryPrediction, double length, double width);

    /**
     * Setter for ID of obstacle.
     *
     * @param obstacleId ID of obstacle.
     */
    void setId(size_t obstacleId);

    /**
     * Setter for isStatic.
     *
     * @param isStatic Boolean indicating whether the obstacle is static or not.
     */
    void setIsStatic(bool isStatic);

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
     * Setter for own lane.
     *
     * @param possibleLanes Lanes which should be considered for finding possible assigned lane.
     * @param timeStep Current time step.
     */
    void setOwnLane(const std::vector<std::shared_ptr<Lane>> &possibleLanes, size_t timeStep);

    /**
     * Setter for reference lane.
     *
     * @param lane Lane which should be used as reference lane for curvilinear coordinate system. Usually, this is the
     * ego vehicle's lane.
     * @param timeStep Current time step.
     */
    void setReferenceLane(const std::shared_ptr<Lane> &lane); // TODO add unit test

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
     * Getter for assigned own lane.
     *
     * @return Pointer to lane object.
     */
    [[nodiscard]] std::shared_ptr<Lane> getOwnLane() const;

    /**
     * Getter for reference lane.
     *
     * @return Pointer to lane object.
     */
    [[nodiscard]] std::shared_ptr<Lane> getReferenceLane() const;

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
    getOccupiedLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

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
    [[nodiscard]] size_t getTrajectoryLength();

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes)
     *
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle front
     */
    double frontS(size_t timeStep);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes)
     *
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle front
     */
    double rearS(size_t timeStep);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and assigned lane
     *
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(size_t timeStep) const;

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned lane
     *
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(size_t timeStep) const;

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and assigned lane
     *
     * @param timeStep time step of interest
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(size_t timeStep) const; // Todo create test case

    /**
     * Extracts the lanes from the road network the obstacle occupies at a certain time step
     *
     * @param timeStep time step of interest
     * @return list of pointers to occupied lanes
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lane>> getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                      size_t timeStep); // TODO create test case

    /**
     * Extracts first time step of trajectory
     *
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] size_t getFirstTrajectoryTimeStep(); // TODO create test case

    /**
     * Extracts last time step of trajectory
     *
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] size_t getLastTrajectoryTimeStep(); // TODO create test case

    /**
     * Converts the x- and y-coordinate into the Curvilinear domain.
     *
     * @param timeStep Time step for which the coordinates should me transformed.
     */
    void convertPointToCurvilinear(size_t timeStep) const;

    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getStraightOutgoings() const;

    void setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straightOutgoings);

    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLeftOutgoings() const;

    void setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &leftOutgoings);

    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getRightOutgoings() const;

    void setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &rightOutgoings);

    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getOncomings() const;

    void setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomings);

    bool timeStepExists(size_t timeStep);

    void interpolateAcceleration(size_t timeStep);

  private:
    size_t id{};                                      //**< unique ID of lanelet */
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
        occupiedLanelets{};                       //**< map of time steps to lanelets occupied by the obstacle */
    std::shared_ptr<Lane> ownLane{nullptr};       //**< lane to which obstacle is assigned to */
    std::shared_ptr<Lane> referenceLane{nullptr}; //**< lane which is used as reference for curvilinear projection */
    std::map<size_t, std::vector<std::shared_ptr<Lane>>>
        occupiedLanes{}; //**< map of time steps to lanes occupied by the obstacle */
    std::vector<std::shared_ptr<Lanelet>>
        straightOutgoings; //**< set of pointers to straight outgoing lanelets to which obstacle belongs */
    std::vector<std::shared_ptr<Lanelet>>
        leftOutgoings; //**< set of pointers to left outgoing lanelets to which obstacle belongs */
    std::vector<std::shared_ptr<Lanelet>>
        rightOutgoings; //**< set of pointers to right outgoing lanelets to which obstacle belongs */
    std::vector<std::shared_ptr<Lanelet>>
        oncomings; //**< set of pointers to oncoming lanelets to which obstacle belongs */
};
