//
// Created by Sebastian Maierhofer on 04.11.20.
//

#ifndef ENV_MODEL_COMMONROAD_FACTORY_2020A_H
#define ENV_MODEL_COMMONROAD_FACTORY_2020A_H

#include "commonroad_factory.h"

class CommonRoadFactory2020a : public CommonRoadFactory {

public:
    explicit CommonRoadFactory2020a(std::unique_ptr<pugi::xml_document> xmlDocument)
            : CommonRoadFactory(std::move(xmlDocument)) {}

    std::vector<std::shared_ptr<Obstacle>> createObstacles() override;
    std::vector<std::shared_ptr<Lanelet>> createLanelets(
            std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
            std::vector<std::shared_ptr<TrafficLight>> trafficLights) override;
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() override;
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() override;
    std::vector<std::shared_ptr<Intersection>> createIntersections(
            const std::vector<std::shared_ptr<Lanelet>>& lanelets) override;

};

#endif //ENV_MODEL_COMMONROAD_FACTORY_2020A_H
