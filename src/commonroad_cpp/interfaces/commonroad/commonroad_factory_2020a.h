#pragma once

#include "commonroad_factory.h"

/**
 * Class for reading a CommonRoad 2020a file.
 */
class CommonRoadFactory2020a : public CommonRoadFactory {

  public:
    /**
     * Constructor for CommonRoadFactory 2020a.
     *
     * @param xmlDocument Pointer to loaded XML.
     */
    explicit CommonRoadFactory2020a(std::unique_ptr<pugi::xml_document> xmlDocument)
        : CommonRoadFactory(std::move(xmlDocument)) {}

    /**
     * Function for creating obstacles.
     *
     * @return List of pointers to created obstacles.
     */
    std::vector<std::shared_ptr<Obstacle>> createObstacles() override;

    /**
     * Function for creating lanelets.
     *
     * @param trafficSigns List of pointers to traffic signs.
     * @param trafficLights List of pointers to traffic lights.
     * @return List of pointers to created lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>>
    createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                   std::vector<std::shared_ptr<TrafficLight>> trafficLights) override;

    /**
     * Function for creating traffic signs.
     *
     * @return List of pointers to created traffic signs.
     */
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() override;

    /**
     * Function for creating traffic lights.
     *
     * @return List of pointers to created traffic lights.
     */
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() override;

    /**
     * Function for creating intersections.
     *
     * @param lanelets List of pointers to created lanelets.
     * @return List of pointers to created intersection.
     */
    std::vector<std::shared_ptr<Intersection>>
    createIntersections(const std::vector<std::shared_ptr<Lanelet>> &lanelets) override;

    /**
     * Function for extracting the time step size.
     *
     * @return Time step size.
     */
    double getTimeStepSize() override;
};
