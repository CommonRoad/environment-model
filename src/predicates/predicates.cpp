//
// Created by Sebastian Maierhofer on 16.01.21.
//

#include "predicates.h"

Predicates::Predicates(std::shared_ptr<RoadNetwork> roadNetwork, SupportedTrafficSignCountry country) : roadNetwork(std::move(roadNetwork)), country(country) {}

void Predicates::setRoadNetwork(const std::shared_ptr<RoadNetwork> &net) { roadNetwork = net; }