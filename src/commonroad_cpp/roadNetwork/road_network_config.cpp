#include "commonroad_cpp/roadNetwork/road_network_config.h"

#include <cassert>

void RoadNetworkParameters::checkParameterValidity() {
    assert(eps1 > 0.0);
    assert(eps2 > 0.0);
    assert(stepsToResamplePolyline > 0);
    assert(cornerCuttingRefinements > 0);
    assert(projectionDomainLimit > 0.0);
}

std::string RoadNetworkParameters::to_string() const {
    return "eps1: " + std::to_string(eps1) + ", eps2: " + std::to_string(eps2) +
           ", stepsToResamplePolyline: " + std::to_string(stepsToResamplePolyline) +
           ", cornerCuttingRefinements: " + std::to_string(cornerCuttingRefinements) +
           ", projectionDomainLimit: " + std::to_string(projectionDomainLimit) +
           ", numAdditionalSegmentsCCS: " + std::to_string(numAdditionalSegmentsCCS) +
           ", numIntersectionsPerDirectionLaneGeneration: " +
           std::to_string(numIntersectionsPerDirectionLaneGeneration) +
           ", relevantTimeIntervalSize: " + std::to_string(relevantTimeIntervalSize);
}
