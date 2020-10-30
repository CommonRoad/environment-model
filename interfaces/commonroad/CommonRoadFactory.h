#pragma once

#include "pugi_xml/pugixml.hpp"
//#include "../world/obstacle/obstacle.h"
#include "../../road_network/lanelet/lanelet.h"
#include <stdexcept>

#include <vector>

using std::vector;

class CommonRoadFactory {

  public:
	CommonRoadFactory(std::unique_ptr<pugi::xml_document> xmlDocument) { doc = std::move(xmlDocument); }

//	virtual vector<std::shared_ptr<obstacle>> createObstacles(double timeStamp, const obstacleParameters *param) = 0;

	virtual vector<std::shared_ptr<Lanelet>> createLanelets() = 0;

  protected:
	std::unique_ptr<pugi::xml_document> doc;
	// pugi::xml_document doc;
};
