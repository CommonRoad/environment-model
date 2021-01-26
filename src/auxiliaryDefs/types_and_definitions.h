//
// Created by Sebastian Maierhofer on 07.11.20.
//

#ifndef ENV_MODEL_TYPES_AND_DEFINITIONS_H
#define ENV_MODEL_TYPES_AND_DEFINITIONS_H

#include <iostream>
#include "vector"
#include "memory"

enum class ObstacleType {
    car, truck, pedestrian, bus, unknown, vehicle
};

enum class LineMarking {
    solid, dashed, broad_dashed, broad_solid, unknown, no_marking
};

enum class LaneletType {
    interstate, urban, crosswalk, busStop, country, highway, driveWay, mainCarriageWay,
    accessRamp, exitRamp, shoulder, busLane, bikeLane, sidewalk, unknown, intersection
};

enum class CycleElementType {
    red, green, yellow, red_yellow
};

enum class TrafficLightDirection {
    right, straight, left, leftStraight, straightRight, leftRight, all
};

enum class DrivingDirection {
    same, opposite, invalid
};

enum class ShapeType {
    rectangle, circle
};

enum class ContainmentType {
    PARTIALLY_CONTAINED, COMPLETELY_CONTAINED
};

const double reactionTimeObstacles = 0.3;

#endif //ENV_MODEL_TYPES_AND_DEFINITIONS_H

