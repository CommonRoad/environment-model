#pragma once

class OutgoingGroup {
  public:
    /**
     * Default constructor.
     */
    OutgoingGroup() = default;

    /**
     * TODO
     * @param groupID
     * @param outgoingLanelets
     */
    OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets);

  private:
    size_t groupID{0};
    std::vector<std::shared_ptr<Lanelet>> outgoingLanelets; /**< set of pointers to lanelets belonging to outgoing group */
};
