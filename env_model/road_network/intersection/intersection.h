//
// Created by sebastian on 01.11.20.
//

#ifndef ENV_MODEL_INTERSECTION_H
#define ENV_MODEL_INTERSECTION_H

#include "incoming.h"

class Intersection {
    public:
        int getId() const;
        const Incoming &getIncoming() const;
        const std::vector<std::vector<std::shared_ptr<Lanelet>>> &getCrossing() const;

        void setId(int id);
        void setIncoming(const Incoming &incoming);
        void setCrossing(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &crossing);

    private:
        int id;
        Incoming incoming;
        std::vector<std::vector<std::shared_ptr<Lanelet>>> crossing;
};


#endif //ENV_MODEL_INTERSECTION_H
