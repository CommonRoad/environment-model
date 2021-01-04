//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_INCOMING_H
#define ENV_MODEL_INCOMING_H

#include "../lanelet/lanelet.h"

/**
 * Class representing an incoming of an intersection.
 */
class Incoming {
public:
    /**
     * Getter for incoming ID.
     *
     * @return Id of incoming.
     */
    [[nodiscard]] int getId() const;

    /**
     * Getter for lanelets belonging to incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getIncomingLanelet() const;

    /**
     * Getter for successor right lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsRight() const;

    /**
     * Getter for successor straight lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsStraight() const;

    /**
     * Getter for successor left lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsLeft() const;

    /**
     * Getter for incoming which is left of this incoming.
     *
     * @return Pointer to incoming.
     */
    [[nodiscard]] const std::shared_ptr<Incoming> &getIsLeftOf() const;

    /**
     * Setter of incoming ID.
     *
     * @param id Id of incoming.
     */
    void setId(int id);

    /**
     * Setter for lanelets belonging to incoming.
     *
     * @param incomingLanelet List of pointers to lanelets.
     */
    void setIncomingLanelet(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelet);

    /**
     * Setter for successor right lanelets.
     *
     * @param successorsRight List of pointers to successor right lanelets.
     */
    void setSuccessorsRight(const std::vector<std::shared_ptr<Lanelet>> &successorsRight);

    /**
     * Setter for successor straight lanelets.
     *
     * @param successorsStraight List of pointers to successor straight lanelets.
     */
    void setSuccessorsStraight(const std::vector<std::shared_ptr<Lanelet>> &successorsStraight);

    /**
     * Setter for successor left lanelets.
     *
     * @param successorsLeft List of pointers to successor left lanelets.
     */
    void setSuccessorsLeft(const std::vector<std::shared_ptr<Lanelet>> &successorsLeft);

    /**
     * Setter for incoming which is left of this incoming.
     *
     * @param isLeftOf Pointer to incoming.
     */
    void setIsLeftOf(const std::shared_ptr<Incoming> &isLeftOf);

private:
    int id;
    std::vector<std::shared_ptr<Lanelet>> incomingLanelet;          //**< set of pointers to lanelets belonging to incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsRight;          //**< set of pointers to successor right lanelets of incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsStraight;       //**< set of pointers to successor straight lanelets of incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsLeft;           //**< set of pointers to successor left lanelets of incoming */
    std::shared_ptr<Incoming> isLeftOf;                             //**< pointer to incoming which is left */

};


#endif //ENV_MODEL_INCOMING_H
