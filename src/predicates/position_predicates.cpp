//
// Created by Sebastian Maierhofer on 13.11.20.
//

#include "predicates.h"


bool Predicates::inFrontOf(int timeStep,
                           const std::shared_ptr<Obstacle> &obsP,
                           const std::shared_ptr<Obstacle> &obsK) {
    if (obsP->frontS(timeStep) < obsK->rearS(timeStep))
        return true;
    else
        return false;
}




