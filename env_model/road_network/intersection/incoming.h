//
// Created by sebastian on 01.11.20.
//

#ifndef ENV_MODEL_INCOMING_H
#define ENV_MODEL_INCOMING_H

#include "../lanelet/lanelet.h"

class Incoming {
    public:
        int getId() const;
        const std::vector<std::shared_ptr<Lanelet>> &getIncomingLanelet() const;
        const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsRight() const;
        const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsStraight() const;
        const std::vector<std::shared_ptr<Lanelet>> &getSuccessorsLeft() const;
        const std::shared_ptr<Incoming> &getIsLeftOf() const;

        void setId(int id);
        void setIncomingLanelet(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelet);
        void setSuccessorsRight(const std::vector<std::shared_ptr<Lanelet>> &successorsRight);
        void setSuccessorsStraight(const std::vector<std::shared_ptr<Lanelet>> &successorsStraight);
        void setSuccessorsLeft(const std::vector<std::shared_ptr<Lanelet>> &successorsLeft);
        void setIsLeftOf(const std::shared_ptr<Incoming> &isLeftOf);

    private:
        int id;
        std::vector<std::shared_ptr<Lanelet>> incomingLanelet;
        std::vector<std::shared_ptr<Lanelet>> successorsRight;
        std::vector<std::shared_ptr<Lanelet>> successorsStraight;
        std::vector<std::shared_ptr<Lanelet>> successorsLeft;
        std::shared_ptr<Incoming> isLeftOf;

};


#endif //ENV_MODEL_INCOMING_H
