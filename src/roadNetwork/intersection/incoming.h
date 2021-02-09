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
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getIncomingLanelets() const;

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
    void setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelets);

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

    /**
     * Getter for straight outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getStraightOutgoings() const;

    /**
     * Setter for straight outgoing lanelets of incoming.
     *
     * @param straightOutgoings List of pointers to lanelets.
     */
    void setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straightOutgoings);

    /**
     * Getter for left outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLeftOutgoings() const;

    /**
     * Setter for left outgoing lanelets of incoming.
     *
     * @param leftOutgoings List of pointers to lanelets.
     */
    void setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &leftOutgoings);

    /**
     * Getter for right outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getRightOutgoings() const;

    /**
     * Setter for right outgoing lanelets of incoming.
     *
     * @param rightOutgoings List of pointers to lanelets.
     */
    void setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &rightOutgoings);

    /**
     * Getter for oncoming lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getOncomings() const;

    /**
     * Setter for oncoming lanelets of incoming.
     *
     * @param oncomings List of pointers to lanelets.
     */
    void setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomings);

private:
    int id;
    std::vector<std::shared_ptr<Lanelet>> incomingLanelets;          //**< set of pointers to lanelets belonging to incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsRight;          //**< set of pointers to successor right lanelets of incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsStraight;       //**< set of pointers to successor straight lanelets of incoming */
    std::vector<std::shared_ptr<Lanelet>> successorsLeft;           //**< set of pointers to successor left lanelets of incoming */
    std::shared_ptr<Incoming> isLeftOf;                             //**< pointer to incoming which is left */
    std::vector<std::shared_ptr<Lanelet>> straightOutgoings;        //**< set of pointers to straight outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>> leftOutgoings;            //**< set of pointers to left outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>> rightOutgoings;           //**< set of pointers to right outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>> oncomings;                //**< set of pointers to oncoming lanelets of this incoming */
};


#endif //ENV_MODEL_INCOMING_H
