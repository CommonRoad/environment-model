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
    [[nodiscard]] const std::vector<std::shared_ptr<Incoming>> &getIncoming() const;

    /**
     * Getter for crossing lanelets belonging to intersection.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::vector<std::shared_ptr<Lanelet>>> &getCrossing() const;

    /**
     * Getter for intersection ID.
     *
     * @param num ID of intersection.
     */
    void setId(int num);

    /**
     * Adds and incoming to the intersection.
     *
     * @param incoming Pointer to incoming.
     */
    void addIncoming(const std::shared_ptr<Incoming> &incoming);

    /**
     * Setter for crossing belonging to intersection.
     *
     * @param crossing List of pointers to lanelets.
     */
    void setCrossing(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &crossing);

private:
    int id;                                                         //**< ID of intersection. */
    std::vector<std::shared_ptr<Incoming>> incoming;                //**< List of pointers to incomings belonging to intersection. */
    std::vector<std::vector<std::shared_ptr<Lanelet>>> crossing;    //**< List of pointers to crossing lanelets belonging to intersection. */
};


#endif //ENV_MODEL_INTERSECTION_H
