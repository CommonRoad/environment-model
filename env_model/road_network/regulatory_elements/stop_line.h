//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_STOP_LINE_H
#define ENV_MODEL_STOP_LINE_H

#include "traffic_sign.h"
#include "traffic_light.h"
#include "../../auxiliaryDefs/types_and_definitions.h"

class StopLine {
    public:
        [[nodiscard]] const std::vector<vertice> &getPoints() const;
        [[nodiscard]] std::shared_ptr<TrafficSign> getTrafficSign() const;
        [[nodiscard]] std::shared_ptr<TrafficLight> getTrafficLight() const;
        [[nodiscard]] LineMarking getLineMarking() const;

        void setPoints(const std::vector<vertice> &points);
        void setTrafficSign(std::shared_ptr<TrafficSign> trafficSign);
        void setTrafficLight(std::shared_ptr<TrafficLight> trafficLight);
        void setLineMarking(LineMarking lineMarking);

    private:
        std::vector<vertice> points;
        std::shared_ptr<TrafficSign> trafficSign;
        std::shared_ptr<TrafficLight> trafficLight;
        LineMarking lineMarking;
};

#endif //ENV_MODEL_STOP_LINE_H
