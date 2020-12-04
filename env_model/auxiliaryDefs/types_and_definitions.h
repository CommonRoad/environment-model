//
// Created by Sebastian Maierhofer on 07.11.20.
//

#ifndef ENV_MODEL_TYPES_AND_DEFINITIONS_H
#define ENV_MODEL_TYPES_AND_DEFINITIONS_H

#include <iostream>
#include "vector"
#include "memory"

enum class ObstacleType{car, truck, pedestrian, bus, unknown, vehicle};

enum class LineMarking{solid, dashed, broad_dashed, broad_solid, unknown, no_marking};

enum class LaneletType{interstate, urban, crosswalk, busStop, country, highway, driveWay, mainCarriageWay,
    accessRamp, exitRamp, shoulder, busLane, bikeLane, sidewalk, unknown};

enum class CycleElementType{red, green, yellow, red_yellow };

enum class TrafficLightDirection{right, straight, left, leftStraight, straightRight, leftRight, all };

enum class DrivingDirection{same, opposite, invalid};

const int PARTIALLY_CONTAINED = 1;
const int COMPLETELY_CONTAINED = 2;

#endif //ENV_MODEL_TYPES_AND_DEFINITIONS_H
