#include "CommonRoadFactory.h"

class CommonRoadFactory2020a : public CommonRoadFactory {

  public:
	CommonRoadFactory2020a(std::unique_ptr<pugi::xml_document> xmlDocument)
		: CommonRoadFactory(std::move(xmlDocument)) {}

//	std::vector<std::shared_ptr<obstacle>> createObstacles(double timeStamp, const obstacleParameters *param) override;
	std::vector<std::shared_ptr<Lanelet>> createLanelets() override;
};
