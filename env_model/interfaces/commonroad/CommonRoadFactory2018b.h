#include "CommonRoadFactory.h"

class CommonRoadFactory2018b : public CommonRoadFactory {

  public:
	CommonRoadFactory2018b(std::unique_ptr<pugi::xml_document> xmlDocument)
		: CommonRoadFactory(std::move(xmlDocument)) {}

	std::vector<std::shared_ptr<Obstacle>> createObstacles(double timeStamp, const obstacleParameters *param) override;
	std::vector<std::shared_ptr<Lanelet>> createLanelets() override;
    std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() override;
};
