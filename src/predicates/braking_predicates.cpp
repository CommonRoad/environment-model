//
// Created by Sebastian Maierhofer on 15.12.20.
//

#include "predicates.h"

#include <cmath>

double
Predicates::safeDistance(double vFollow, double vLead, double aMinFollow, double aMinLead, double tReact) {
    return pow(vLead, 2) / (-2 * std::abs(aMinLead)) - pow(vFollow, 2) / (-2 * std::abs(aMinFollow)) + vFollow * tReact;
}

bool Predicates::keepsSafeDistancePrec(int timeStep,
                                              const std::shared_ptr<Obstacle> &vehicleFollow,
                                              const std::shared_ptr<Obstacle> &vehicleLead) {
    double aMinFollow{vehicleFollow->getAminLong()};
    double aMinLead{vehicleLead->getAminLong()};
    double tReact{vehicleFollow->getReactionTime()};
    double dSafe{safeDistance(vehicleFollow->getTrajectoryPrediction().at(timeStep).getVelocity(),
                              vehicleLead->getTrajectoryPrediction().at(timeStep).getVelocity(),
                              aMinFollow, aMinLead, tReact)};
    double deltaS = vehicleLead->rearS(timeStep) - vehicleFollow->frontS(timeStep);
    if (0 <= deltaS < dSafe)
        return false;
    else
        return true;
}

void Predicates::setRoadNetwork(const std::shared_ptr<RoadNetwork> &net) { roadNetwork = net; }
