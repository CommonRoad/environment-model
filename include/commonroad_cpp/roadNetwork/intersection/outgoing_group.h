#pragma once

#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

class OutgoingGroup {
  public:
    /**
     * Default constructor.
     */
    OutgoingGroup() = default;

    /**
     * Constructor
     *
     * @param groupID if of OutgoingGroup
     * @param outgoingLanelets vector of pointers to Lanelets of outgoingGroup
     */
    OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets);

    /**
     * Setter for the ID
     *
     * @param groupId size_t
     */
    void setId(size_t outId);

    /**
     * Getter for the ID
     */
    size_t getId() const;

    /**
     * Add pointer to Lanelet to outgoingLanelets of OutgoingGroup
     *
     * @param lanelet poiter to Lanelet
     */
    void addOutgoingLanelet(std::shared_ptr<Lanelet> &lanelet);

    /**
     * Setter for lanelets belonging to outgoingGroup
     *
     * @param outgoingLanelets pointer to vector of lanelets
     */
    void setOutgoingLanelets(const std::vector<std::shared_ptr<Lanelet>> &outLanelets);

    /**
     * Getter for outgoingLanelets of OutgoingGroup
     *
     * @return vector of pointer to Lanelets
     */
    std::vector<std::shared_ptr<Lanelet>> getOutgoingLanelets();

  private:
    size_t groupID{0};
    std::optional<size_t> incomingGroupID;
    std::vector<std::shared_ptr<Lanelet>> outgoingLanelets; /**< set of pointers to lanelets belonging to outgoing group */
};
