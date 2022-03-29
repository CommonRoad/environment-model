//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "obstacle.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"

namespace obstacle_operations {

/**
 * Returns the obstacle which corresponds to a given obstacle ID.
 *
 * @param obstacleId obstacle ID
 * @return pointer to obstacle
 */
std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                          size_t obstacleId);

/**
 * Matches a string to an obstacle type
 *
 * @param string for which obstacle type should be extracted
 * @return obstacle type which corresponds to string or unknown type if string does not match
 */
ObstacleType matchStringToObstacleType(const std::string &type);

/**
 * Computes obstacle which is directly left of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which directly left obstacle is computed.
 * @param roadNetwork Relevant road network
 * @return Obstacle directly left.
 */
std::shared_ptr<Obstacle> obstacleDirectlyLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes all obstacles which are left of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which left obstacles are computed.
 * @param roadNetwork Relevant road network
 * @return List of obstacles located left of given obstacle.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesLeft(size_t timeStep,
                                                     const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes all obstacle which are left or right adjacent to a given obstacle k based on kth reference path.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which adjacent obstacles are computed.
 * @param roadNetwork Relevant road network
 * @return List of adjacent obstacles.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesAdjacent(size_t timeStep,
                                                         const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes obstacle which is directly right of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which directly right obstacle is computed.
 * @param roadNetwork Relevant road network
 * @return Obstacle directly right.
 */
std::shared_ptr<Obstacle> obstacleDirectlyRight(size_t timeStep,
                                                const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes all obstacles which are right of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which right obstacles are computed.
 * @param roadNetwork Relevant road network
 * @return List of obstacles located right of given obstacle.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesRight(size_t timeStep,
                                                      const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes the set of lanelets on the right side of the obstacle not including occupied lanelets.
 *
 * @param timeStep Time step of interest.
 * @param obs Obstacle based on which right lanelets are computed.
 * @param roadNetwork Relevant road network
 * @return Set of lanelets on the right side of the obstacle.
 */
std::set<std::shared_ptr<Lanelet>> laneletsRightOfObstacle(size_t timeStep,
                                                           const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                           const std::shared_ptr<Obstacle> &obs);

/**
 * Computes the set of lanelets on the left side of the obstacle not including occupied lanelets.
 *
 * @param timeStep Time step of interest.
 * @param obs Obstacle based on which left lanelets are computed.
 * @param roadNetwork Relevant road network
 * @return Set of lanelets on the left side of the obstacle.
 */
std::set<std::shared_ptr<Lanelet>> laneletsLeftOfObstacle(size_t timeStep,
                                                          const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                          const std::shared_ptr<Obstacle> &obs);


std::vector<std::shared_ptr<Intersection>> getIntersections(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                                 const std::shared_ptr<Obstacle> &obs);

} // namespace obstacle_operations