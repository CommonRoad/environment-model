//
// Created by Sebastian Maierhofer on 16.01.21.
//

#include "predicates.h"

bool Predicates::cutIn(int timeStep, const  std::shared_ptr<Obstacle> &obsK, const  std::shared_ptr<Obstacle> &obsP){
    // only one lane occupied -> no cut-in possible
    if (obsK->getOccupiedLanes(roadNetwork, timeStep).size() == 1)
        return false;
    // the two vehicles do not occupy the same lane -> no cut-in of the kth obstacle into the pth obstacle's lane
    if (!inSameLane(timeStep, obsK, obsP))
        return false;
    // check orientation and lateral distance
    if (obsK->getLatPosition(timeStep) < obsP->getLatPosition(timeStep)
    && obsK->getStateByTimeStep(timeStep)->getOrientation() > 0
    || (obsK->getStateByTimeStep(timeStep)->getLatPosition() > obsP->getLatPosition(timeStep)
    and obsK->getStateByTimeStep(timeStep)->getOrientation() < 0))
        return true;
    else
        return false;
}

bool Predicates::makesUTurn(int timeStep, const  std::shared_ptr<Obstacle> &obs){
    auto lanes = obs->getOccupiedLanes(roadNetwork, timeStep);
    if (std::any_of(lanes.begin(), lanes.end(),
                    [obs, timeStep](const std::shared_ptr<Lane>& la)
                    {return 1.57 <= abs(obs->getStateByTimeStep(timeStep)->getOrientation() -   // TODO use parameter
                     la->getLanelet().getOrientationAtPosition(obs->getStateByTimeStep(timeStep)->getXPosition(),
                                                              obs->getStateByTimeStep(timeStep)->getYPosition()));
                     }))
            return true;
    return false;
}

