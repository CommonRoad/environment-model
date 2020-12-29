//
// Created by Sebastian Maierhofer on 31.10.20.
//

#ifndef ENV_MODEL_TRAFFIC_LIGHT_H
#define ENV_MODEL_TRAFFIC_LIGHT_H

#include "../../auxiliaryDefs/structs.h"

class TrafficLight {
    public:
        TrafficLight();                                     // default constructor
        /*
         * setter functions
         */
        void setId(int num);
        void setCycle(const std::vector<CycleElement>& light_cycle);
        void setOffset(int ofst);
        void setDirection(TrafficLightDirection dir);
        void setActive(bool ac);
        void addCycleElement(CycleElement ce);

        /*
         * getter functions
         */
        [[nodiscard]] int getId() const;
        [[nodiscard]] std::vector<CycleElement> getCycle() const;
        [[nodiscard]] int getOffset() const;
        [[nodiscard]] TrafficLightDirection getDirection() const;
        [[nodiscard]] bool isActive() const;

        CycleElement getElementAtTime(int time);

    private:
        int id;                          //unique id
        std::vector<CycleElement> cycle;    //cycle of the traffic light
        int offset;                       //the offset for the cycle
        TrafficLightDirection direction;
        bool active{};
};

#endif //ENV_MODEL_TRAFFIC_LIGHT_H
