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
     * @param startPoint start vertex of stop line.
     * @param endPoint end vertex of stop line.
     * @param traffic_sign List of pointers to associated traffic signs.
     * @param traffic_light List of pointers to associated traffic lights.
     * @param line_marking Line marking type of stop line.
     */
    StopLine(vertex startPoint, vertex endPoint, LineMarking line_marking);

    /**
     * Default constructor.
     */
    StopLine() = default;

    /**
     * Getter for start and end point of stop line.
     *
     * @return Start and end vertex.
     */
    std::vector<vertex> getPoints();

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
     * Setter for line marking of stop line.
     *
     * @param lineMarking Type of line marking.
     */
    void setLineMarking(LineMarking lineMarking);

  private:
    vertex startPoint;                                        //**< start vertex of stop line */
    vertex endPoint;                                          //**< end vertex of stop line */
    LineMarking lineMarking;                                  //**< type of line marking */
};
