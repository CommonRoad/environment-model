#include "CommonRoadFactory.h"

class CommonRoadFactory2020a : public CommonRoadFactory {

  public:
	explicit CommonRoadFactory2020a(std::unique_ptr<pugi::xml_document> xmlDocument)
		: CommonRoadFactory(std::move(xmlDocument)) {}

	std::vector<std::shared_ptr<Obstacle>> createObstacles(double timeStamp, const obstacleParameters *param) override;
	std::vector<std::shared_ptr<Lanelet>> createLanelets() override;
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() override;
    std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() override;
};
