//
// Created by Sebastian Maierhofer on 23.01.21.
//

#include "predicates.h"

bool Predicates::reverse(int timeStep, const std::shared_ptr<Obstacle> &obs){
    if (obs->getStateByTimeStep(timeStep).getVelocity() < -0.01)  // TODO add parameter
        return true;
    else
        return false;
}
