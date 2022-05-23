//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "traffic_light.h"
#include "traffic_sign.h"

/**
 * Class representing a CommonRoad stop line.
 */
class StopLine {
  public:
    /**
     * Constructor.
     *
     * @param points Left and right vertex of stop line.
     * @param traffic_sign List of pointers to associated traffic signs.
     * @param traffic_light List of pointers to associated traffic lights.
     * @param line_marking Line marking type of stop line.
     */
    StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficSign>> traffic_sign,
             std::vector<std::shared_ptr<TrafficLight>> traffic_light, LineMarking line_marking);

    /**
     * Constructor.
     *
     * @param points Left and right vertex of stop line.
     * @param traffic_sign List of pointers to associated traffic signs.
     * @param line_marking Line marking type of stop line.
     */
    StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficSign>> traffic_sign,
             LineMarking line_marking);

    /**
     * Constructor.
     *
     * @param points Left and right vertex of stop line.
     * @param traffic_light List of pointers to associated traffic lights.
     * @param line_marking Line marking type of stop line.
     */
    StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficLight>> traffic_light,
             LineMarking line_marking);

    /**
     * Default constructor.
     */
    StopLine() = default;

    /**
     * Getter for start and end point of stop line.
     *
     * @return Start and end vertex.
     */
    [[nodiscard]] const std::vector<vertex> &getPoints() const;

    /**
     * Getter for traffic signs referenced by stop line.
     *
     * @return List of pointers to traffic signs.
     */
    [[nodiscard]] std::vector<std::shared_ptr<TrafficSign>> getTrafficSigns() const;

    /**
     * Getter for traffic lights referenced by stop line.
     *
     * @return List of pointers to traffic lights.
     */
    [[nodiscard]] std::vector<std::shared_ptr<TrafficLight>> getTrafficLights() const;

    /**
     * Getter for line marking type of stop line.
     *
     * @return Line marking type.
     */
    [[nodiscard]] LineMarking getLineMarking() const;

    /**
     * Setter for start and end vertex of stop line.
     *
     * @param points Start and end vertex.
     */
    void setPoints(const std::vector<vertex> &points);

    /**
     * Setter for traffic signs referenced by stop line.
     *
     * @param trafficSign List of pointers to traffic signs.
     */
    void setTrafficSigns(std::vector<std::shared_ptr<TrafficSign>> trafficSign);

    /**
     * Setter for traffic lights referenced by stop line.
     *
     * @param trafficLight List of pointers to traffic lights.
     */
    void setTrafficLights(std::vector<std::shared_ptr<TrafficLight>> trafficLight);

    /**
     * Adds single traffic sign referenced by stop line.
     *
     * @param trafficSign Pointer to traffic sign.
     */
    void addTrafficSign(const std::shared_ptr<TrafficSign> &trafficSign);

    /**
     * Adds traffic light referenced by stop line.
     *
     * @param trafficLight Pointer to traffic light.
     */
    void addTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight);

    /**
     * Setter for line marking of stop line.
     *
     * @param lineMarking Type of line marking.
     */
    void setLineMarking(LineMarking lineMarking);

  private:
    std::vector<vertex> points;                               //**< start and end vertex of stop line */
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;   //**< pointer to traffic signs referenced by stop line */
    std::vector<std::shared_ptr<TrafficLight>> trafficLights; //**< pointer to traffic lights referenced by stop line */
    LineMarking lineMarking;                                  //**< type of line marking */
};
