//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/roadNetwork/lanelet/lanelet_graph.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "incoming_group.h"
#include "outgoing_group.h"

/**
 * Class representing an intersection.
 */
class Intersection {
  public:
    /**
     * Default constructor of intersection class.
     */
    Intersection() = default;

    /**
     * Constructor of intersection class.
     * @param intersectionId Id of intersection.
     * @param incomingGroups List of references to ingoingGroups of intersection.
     * @param outgoingGroups List of references to outgoingGroups of intersection.
     */
    Intersection(size_t intersectionId, std::vector<std::shared_ptr<IncomingGroup>> incomingGroups,
                 std::vector<std::shared_ptr<OutgoingGroup>> outgoingGroups);

    /**
     * Getter for intersection ID.
     *
     * @return ID of intersection.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for incomingGroups belonging to intersection.
     *
     * @return List of pointers to IncomingGroups.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<IncomingGroup>> &getIncomingGroups() const;

    /**
     * Getter for outgoinGroups belonging to intersection.
     *
     * @return List of pointers to OutcomingGroups.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<OutgoingGroup>> &getOutgoingGroups() const;

    /**
     * Getter for intersection ID.
     *
     * @param num ID of intersection.
     */
    void setId(size_t num);

    /**
     * Setter for incomingGroups.
     *
     * @param incs List of pointers to IncomingGroup.
     */
    void setIncomingGroups(const std::vector<std::shared_ptr<IncomingGroup>> &incs);

    /**
     * Setter for outgoingGroups.
     *
     * @param incs List of pointers to OutgoingGroup.
     */
    void setOutgoingGroups(const std::vector<std::shared_ptr<OutgoingGroup>> &outs);

    /**
     * Adds an incomingGroup to the intersection.
     *
     * @param incoming Pointer to IncomingGroup.
     */
    void addIncomingGroup(const std::shared_ptr<IncomingGroup> &incoming);

    /**
     * Adds an outgoingGroup to the intersection.
     *
     * @param incoming Pointer to OutgoingGroup.
     */
    void addOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoing);

    /**
     * Getter for all member lanelets of the intersection
     * @return vector of pointer to Lanelets
     */
    const std::vector<std::shared_ptr<Lanelet>> &getMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork);

    /**
     * Based on the incomingGroups, this fuction computes the member lanelets of the intersection and sets appropriate
     * LaneletTypes for all member lanelets
     */
    void computeMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork);

  private:
    size_t id;                                        //**< ID of intersection. */
    std::vector<std::shared_ptr<IncomingGroup>> incomings; //**< List of pointers to incomings belonging to intersection. */
    std::vector<std::shared_ptr<OutgoingGroup>> outgoings; //**< List of pointers to outgoiungs belonging to intersection. */
    std::vector<std::shared_ptr<Lanelet>> memberLanelets; //**< List of lanelets belonging to intersection starting from
                                                          // incoming until outgoing. Crossings are not considered. */
};
