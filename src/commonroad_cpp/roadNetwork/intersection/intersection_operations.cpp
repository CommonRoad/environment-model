//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "intersection_operations.h"

bool intersection_operations::onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                         std::shared_ptr<RoadNetwork> roadNetwork) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &inter : roadNetwork->getIntersections()) {
        for (const auto &inc : inter->getIncomings()) {
            for (const auto &la : lanelets)
                if (std::any_of(inc->getIncomingLanelets().begin(), inc->getIncomingLanelets().end(),
                                [la](const std::shared_ptr<Lanelet> &incomingLanelet) {
                                    return la->getId() == incomingLanelet->getId();
                                })) {
                    //            obs->setLeftOutgoings(incom->getLeftOutgoings());
                    //            obs->setStraightOutgoings(incom->getStraightOutgoings());
                    //            obs->setRightOutgoings(incom->getRightOutgoings());
                    return true;
                }
        }
    }
    return false;
}