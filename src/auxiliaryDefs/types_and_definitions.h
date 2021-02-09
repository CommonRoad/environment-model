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
    accessRamp, exitRamp, shoulder, busLane, bikeLane, sidewalk, unknown, intersection, incoming
};

enum class TrafficLightState {
    red, green, yellow, red_yellow, inactive
};

enum class TurningDirections {
    left, straight, right, leftStraight, straightRight, leftRight, all
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
    WARNING_DANGER_SPOT = 101,
    WARNING_RIGHT_BEFORE_LEFT = 102,
    WARNING_STEEP_HILL_DOWNWARDS = 108,
    WARNING_SLIPPERY_ROAD = 114,
    WARNING_CONSTRUCTION_SITE = 123,
    WARNING_CROSSING_CYCLIST = 138,
    WARNING_ANIMAL_CROSSING_RIGHT = 142-10,
    RAILWAY = 201,
    YIELD = 205,
    STOP = 206,
    RIGHT_OF_WAY = 301,
    PRIORITY = 306,
    PRIORITY_OVER_ONCOMING = 308,
    GREEN_ARROW = 720,
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = 100210,
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = 100211,
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = 100212,
    ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = 100213,
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = 100214,
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = 100220,
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = 100221,
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = 100222,
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = 100223,
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = 100224,
};

enum class SupportedTrafficSignCountry {
    GERMANY, ZAMUNDA
};

const double reactionTimeObstacles = 0.3;

const std::map<std::string, std::vector<int>> priorityTable{
        std::pair<std::string, std::vector<int>>("306", {4, 5, 4}),
        std::pair<std::string, std::vector<int>>("301", {4, 5, 4}),
        std::pair<std::string, std::vector<int>>("205", {2, 2, 2}),
        std::pair<std::string, std::vector<int>>("206", {1, 1, 1}),
        std::pair<std::string, std::vector<int>>("102", {3, 3, 3}),
        std::pair<std::string, std::vector<int>>("720", {-1, -1, 0}),
        std::pair<std::string, std::vector<int>>("1002-10", {5, 4, 4}),
        std::pair<std::string, std::vector<int>>("1002-11", {2, 2, 2}),
        std::pair<std::string, std::vector<int>>("1002-12", {5, 4, -1}),
        std::pair<std::string, std::vector<int>>("1002-13", {5, -1, 4}),
        std::pair<std::string, std::vector<int>>("1002-14", {2, 2, -1}),
        std::pair<std::string, std::vector<int>>("1002-20", {4, 4, 5}),
        std::pair<std::string, std::vector<int>>("1002-21", {2, 2, 2}),
        std::pair<std::string, std::vector<int>>("1002-22", {-1, 4, 5}),
        std::pair<std::string, std::vector<int>>("1002-23", {4, -1, 5}),
        std::pair<std::string, std::vector<int>>("1002-24", {-1, 2, 2})
};

#endif //ENV_MODEL_TYPES_AND_DEFINITIONS_H

