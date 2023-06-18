//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>

#include <utility>

std::vector<vertex> StopLine::getPoints() { return {startPoint, endPoint}; }

void StopLine::setPoints(const std::vector<vertex> &pos) {
    startPoint = pos[0];
    startPoint = pos[1];
}

LineMarking StopLine::getLineMarking() const { return lineMarking; }

void StopLine::setLineMarking(LineMarking marking) { lineMarking = marking; }

StopLine::StopLine(vertex startPoint, vertex endPoint, LineMarking line_marking)
    : startPoint(startPoint), endPoint(endPoint), lineMarking(line_marking) {}
