//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_INTERSECTION_H
#define ENV_MODEL_INTERSECTION_H

#include "incoming.h"

/**
 * Class representing an intersection.
 */
class Intersection {
public:
    /**
     * Getter for intersection ID.
     *
     * @return ID of intersection.
     */
    [[nodiscard]] int getId() const;

    /**
     * Getter for incomings belonging to intersection.
     *
     * @return List of pointers to incomings.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Incoming>> &getIncomings() const;

    /**
     * Getter for crossing lanelets belonging to intersection.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getCrossings() const;

    /**
     * Getter for intersection ID.
     *
     * @param num ID of intersection.
     */
    void setId(int num);

    /**
     * Setter for incomings.
     *
     * @param incs List of pointers to incomings.
     */
    void setIncomings(const std::vector<std::shared_ptr<Incoming>> &incs);

    /**
     * Adds and incoming to the intersection.
     *
     * @param incoming Pointer to incoming.
     */
    void addIncoming(const std::shared_ptr<Incoming> &incoming);

    /**
     * Setter for crossing lanelets belonging to intersection.
     *
     * @param cr List of pointers to lanelets.
     */
    void setCrossings(const std::vector<std::shared_ptr<Lanelet>> &cr);

private:
    int id;                                                         //**< ID of intersection. */
    std::vector<std::shared_ptr<Incoming>> incomings;               //**< List of pointers to incomings belonging to intersection. */
    std::vector<std::shared_ptr<Lanelet>> crossings;   //**< List of pointers to crossing lanelets belonging to intersection. */
};


#endif //ENV_MODEL_INTERSECTION_H
