//
// Created by Sebastian Maierhofer on 28.10.20.
//

#ifndef ENV_MODEL_STRUCTS_H
#define ENV_MODEL_STRUCTS_H

#include "types_and_definitions.h"

struct TrafficLightCycleElement {
    TrafficLightState color;
    int duration;
};

struct vertex {
    double x;
    double y;
};

struct ValidStates {
    bool xPosition;
    bool yPosition;
    bool velocity;
    bool acceleration;
    bool lonPosition;
    bool latPosition;
    bool orientation;
};



#endif //ENV_MODEL_STRUCTS_H

