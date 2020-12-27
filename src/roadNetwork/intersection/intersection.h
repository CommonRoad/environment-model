//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_INTERSECTION_H
#define ENV_MODEL_INTERSECTION_H

#include "incoming.h"

class Intersection {
    public:
        [[nodiscard]] int getId() const;
        [[nodiscard]] const std::vector<std::shared_ptr<Incoming>> &getIncoming() const;
        [[nodiscard]] const std::vector<std::vector<std::shared_ptr<Lanelet>>> &getCrossing() const;

        void setId(int id);
        void addIncoming(const std::shared_ptr<Incoming>& incoming);
        void setCrossing(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &crossing);

    private:
        int id;
        std::vector<std::shared_ptr<Incoming>> incoming;
        std::vector<std::vector<std::shared_ptr<Lanelet>>> crossing;
};


#endif //ENV_MODEL_INTERSECTION_H
