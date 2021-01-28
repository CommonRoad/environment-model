//
// Created by Sebastian Maierhofer on 23.01.21.
//

#include "predicates.h"

bool Predicates::reverse(int timeStep, const std::shared_ptr<Obstacle> &obs){
    if (obs->getStateByTimeStep(timeStep)->getVelocity() < -0.01)  // TODO add parameter
        return true;
    else
        return false;
}

bool Predicates::inStandstill(int timeStep, const  std::shared_ptr<Obstacle> &obs){
    if (-0.01 < obs->getStateByTimeStep(timeStep)->getVelocity() and obs->getStateByTimeStep(timeStep)->getVelocity() < 0.01) //TODO add parameters
    {
        return true;
    }
    else
        return false;
}
