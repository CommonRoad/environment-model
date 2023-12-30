//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>

#include <utility>

std::pair<vertex, vertex> StopLine::getPoints() { return points; }

void StopLine::setPoints(const std::pair<vertex, vertex> &position) { points = position; }

LineMarking StopLine::getLineMarking() const { return lineMarking; }

void StopLine::setLineMarking(LineMarking marking) { lineMarking = marking; }

StopLine::StopLine(std::pair<vertex, vertex> points, LineMarking line_marking)
    : points(std::move(points)), lineMarking(line_marking) {}
