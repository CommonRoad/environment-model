//
// Created by Sebastian Maierhofer on 07.11.20.
//

#ifndef ENV_MODEL_TYPES_AND_DEFINITIONS_H
#define ENV_MODEL_TYPES_AND_DEFINITIONS_H

#include <iostream>
#include "vector"
#include "memory"
#include <map>

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

enum class TrafficLightState {
    red, green, yellow, red_yellow, inactive
};

enum class TurningDirections {
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

enum class TrafficSignIDGermany {
    green_arrow = 720
};

enum class SupportedTrafficSignCountry {
    GERMANY, ZAMUNDA
};

const double reactionTimeObstacles = 0.3;

const std::map<std::string, std::tuple<int, int, int>> priorityTable{
        std::pair<std::string, std::tuple<int, int, int>>("306", std::tuple<int, int, int>(4, 5, 4)),
        std::pair<std::string, std::tuple<int, int, int>>("301", std::tuple<int, int, int>(4, 5, 4)),
        std::pair<std::string, std::tuple<int, int, int>>("205", std::tuple<int, int, int>(2, 2, 2)),
        std::pair<std::string, std::tuple<int, int, int>>("206", std::tuple<int, int, int>(1, 1, 1)),
        std::pair<std::string, std::tuple<int, int, int>>("102", std::tuple<int, int, int>(3, 3, 3)),
        std::pair<std::string, std::tuple<int, int, int>>("720", std::tuple<int, int, int>(-1, -1, 0)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-10", std::tuple<int, int, int>(5, 4, 4)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-11", std::tuple<int, int, int>(2, 2, 2)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-12", std::tuple<int, int, int>(5, 4, -1)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-13", std::tuple<int, int, int>(5, -1, 4)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-14", std::tuple<int, int, int>(2, 2, -1)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-20", std::tuple<int, int, int>(4, 4, 5)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-21", std::tuple<int, int, int>(2, 2, 2)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-22", std::tuple<int, int, int>(-1, 4, 5)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-23", std::tuple<int, int, int>(4, -1, 5)),
        std::pair<std::string, std::tuple<int, int, int>>("1002-24", std::tuple<int, int, int>(-1, 2, 2))
};

#endif //ENV_MODEL_TYPES_AND_DEFINITIONS_H

