#include "commonroad_factory.h"

class CommonRoadFactory2018b : public CommonRoadFactory {

  public:
	explicit CommonRoadFactory2018b(std::unique_ptr<pugi::xml_document> xmlDocument)
		: CommonRoadFactory(std::move(xmlDocument)) {}

	std::vector<std::shared_ptr<Obstacle>> createObstacles() override;
	std::vector<std::shared_ptr<Lanelet>> createLanelets() override;
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() override;
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() override;
};
