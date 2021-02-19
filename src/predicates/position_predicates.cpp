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

double Predicates::rearPosition(int timeStep, const std::shared_ptr<Lane> &referenceLane, const std::shared_ptr<Obstacle> &obs) {
    double s = obs->getLonPosition(timeStep);
    double width = obs->getGeoShape().getWidth();
    double length = obs->getGeoShape().getLength();
    double theta = obs->getStateByTimeStep(timeStep)->getOrientation() - referenceLane->
            getLanelet().getOrientationAtPosition(obs->getStateByTimeStep(timeStep)->getXPosition(),
                                                  obs->getStateByTimeStep(timeStep)->getYPosition());

    // use minimum of all corners
    return std::min({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}


double Predicates::frontPosition(int timeStep, const std::shared_ptr<Lane> &referenceLane, const std::shared_ptr<Obstacle> &obs) {
    double s = obs->getLonPosition(timeStep);
    double width = obs->getGeoShape().getWidth();
    double length = obs->getGeoShape().getLength();
    double theta = obs->getStateByTimeStep(timeStep)->getOrientation() - referenceLane->
            getLanelet().getOrientationAtPosition(obs->getStateByTimeStep(timeStep)->getXPosition(),
                                                  obs->getStateByTimeStep(timeStep)->getYPosition());

    // use maximum of all corners
    return std::max({(length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (length / 2) * cos(theta) - (-width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (width / 2) * sin(theta) + s,
                     (-length / 2) * cos(theta) - (-width / 2) * sin(theta) + s});
}


