#pragma once

#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

/**
 * Class representing an incoming of an intersection.
 */
class CrossingGroup {
  public:
    /**
     * Default constructor.
     */
    CrossingGroup() = default;

    /**
     * Constructor of incoming.
     *
     * @param crossingId Incoming ID.
     * @param crossingLanelets Reference to lanelets belonging to incoming.
     * @param incomingGroupId Reference incoming orientated left.
     * @param outgoingGroupId Reference to straight outgoing lanelets.
     */
    CrossingGroup(size_t crossingId, std::vector<std::shared_ptr<Lanelet>> crossingLanelets, size_t incomingGroupId,
                  size_t outgoingGroupId);

    /**
     * Getter for crossing group ID.
     *
     * @return Id of crossing group.
     */
    [[nodiscard]] size_t crossingId() const;

    /**
     * Getter for lanelets belonging to crossing group.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getCrossingGroupLanelets() const;

    /**
     * Setter of crossing group ID.
     *
     * @param crossingId Id of crossing group.
     */
    void crossingGroupId(size_t crossingId);

    /**
     * Setter for lanelets belonging to crossing group.
     *
     * @param crossingLanelets List of pointers to lanelets.
     */
    void setCrossingGroupLanelets(const std::vector<std::shared_ptr<Lanelet>> &crossingLanelets);

    /**
     * Adds crossing lanelet to list of crossing group lanelets.
     *
     * @param crossingLanelet Crossing lanelet
     */
    void addCrossingLanelet(const std::shared_ptr<Lanelet> &crossingLanelet);

    /**
     * Setter for OutgoingGroup
     * @param outId Outgoing group ID.
     */
    void setOutgoingGroupID(size_t outId);

    /**
     * Getter for OutgoingGroup
     * @return  outId Outgoing group ID.
     */
    std::optional<size_t> getOutgoingGroupID();

    /**
     * Setter for IncomingGroup
     * @param incId Incoming group ID.
     */
    void setIncomingGroupID(size_t incId);

    /**
     * Getter for OutgoingGroup
     * @return  incId Incoming group ID.
     */
    std::optional<size_t> getIncomingGroupID();

  private:
    size_t groupID;
    std::optional<size_t> outgoingGroupID;
    std::optional<size_t> incomingGroupID;
    std::vector<std::shared_ptr<Lanelet>> crossingGroupLanelets; //**< set of pointers to lanelets belonging to crossing group */
};
