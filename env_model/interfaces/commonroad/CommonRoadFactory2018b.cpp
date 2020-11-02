#include "CommonRoadFactory2018b.h"
//#include "../world/Obstacle/vehicle.h"

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2018b::createObstacles() {

	std::vector<std::shared_ptr<Obstacle>> obstacleList{};

	pugi::xml_node commonRoad = doc->child("commonRoad");
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
		 roadElements = roadElements.next_sibling()) {
		if (!(strcmp(roadElements.name(), "Obstacle"))) {
			if (!(strcmp(roadElements.first_child().text().as_string(), "dynamic")) &&
				!(strcmp(roadElements.first_child().next_sibling().text().as_string(), "car")))

			{
				bool timeStampAvailable = false;

				std::shared_ptr<Obstacle> tempObstacle(nullptr); // Empty pointer (specific object gets assigned
				// depending on Obstacle type)
//				if (param)
//					tempObstacle = std::make_shared<Obstacle>(
//						param->a_max_vehicles, param->v_max_vehicles, param->v_s_vehicles, param->speedingFactor,
//						param->occM1_vehicles, param->occM2_vehicles, param->occM3_vehicles);
//				else

				tempObstacle = std::make_shared<Obstacle>();

				tempObstacle->setId(roadElements.first_attribute().as_int());
				for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
					if (!(strcmp(child.name(), "shape"))) {
						if (!(strcmp(child.first_child().name(), "rectangle"))) {
							tempObstacle->getGeoShape().setLength(
								child.first_child().child("length").text().as_double());
							tempObstacle->getGeoShape().setWidth(child.first_child().child("width").text().as_double());
						}
						continue;
					}
                    if (!(strcmp(child.name(), "initialState"))) {
                        pugi::xml_node states = child;
                        State initialState;
                        initialState.setTimeStep(0);
                        initialState.setXPosition(states.child("position").child("point").child("x").text().as_double());
                        initialState.setYPosition(states.child("position").child("point").child("y").text().as_double());
                        initialState.setOrientation(states.child("orientation").child("exact").text().as_double());
                        initialState.setVelocity(states.child("velocity").child("exact").text().as_double());
                        initialState.setAcceleration(states.child("acceleration").child("exact").text().as_double());
                        tempObstacle->appendState(initialState);
                    } else if (!(strcmp(child.name(), "trajectory"))) {
                        for (pugi::xml_node states = child.first_child(); states; states = states.next_sibling()) {
                            State st;
                            st.setTimeStep(states.child("time").child("exact").text().as_int());
                            st.setXPosition(states.child("position").child("point").child("x").text().as_double());
                            st.setYPosition(states.child("position").child("point").child("y").text().as_double());
                            st.setOrientation(states.child("orientation").child("exact").text().as_double());
                            st.setVelocity(states.child("velocity").child("exact").text().as_double());
                            st.setAcceleration(states.child("acceleration").child("exact").text().as_double());
                            tempObstacle->appendState(st);
                        }
                    }
				}
                obstacleList.emplace_back(tempObstacle);
//				if (timeStampAvailable) {
//					// if not true --> shared pointer deletes allocated maemory if out of scope
//					// obst->updateInLane(lanes);
//					obstacleList.emplace_back(tempObstacle);
//				}
			}
//			// Todo Add other obstacles than cars
//			else if (!(strcmp(roadElements.first_child().text().as_string(), "static"))) {
//				std::shared_ptr<Obstacle> tempObstacle(nullptr); // Empty pointer (specific object gets assigned
//				// depending on Obstacle type)
//				tempObstacle = std::make_shared<Obstacle>(true);
//				tempObstacle->setId(roadElements.first_attribute().as_int());
//				for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
//					if (!(strcmp(child.name(), "shape"))) {
//						pugi::xml_node var = child.first_child();
//						// tempObstacle->setOrientation(var.child("orientation").text().as_double());
//						// tempObstacle->setPosition(var.child("center").child("x").text().as_double(),
//						//                         var.child("center").child("y").text().as_double());
//						tempObstacle->getGeoShape().setLength(var.child("length").text().as_double());
//						tempObstacle->getGeoShape().setWidth(var.child("width").text().as_double());
//
//					} else if (!(strcmp(child.name(), "initialState"))) {
//						pugi::xml_node states = child;
//						// tempObstacle->setTimeStamp(timeStamp / 10.0);
//						// tempObstacle->setOffset(0.0);
//						tempObstacle->setPosition(
//							states.child("position").child("point").child("x").text().as_double(),
//							states.child("position").child("point").child("y").text().as_double());
//						tempObstacle->setOrientation(states.child("orientation").child("exact").text().as_double());
//					}
//				}
//				// obst->updateInLane(lanes);
//				obstacleList.emplace_back(tempObstacle);
//			}
		}
	}

	return obstacleList;
}
std::vector<std::shared_ptr<Lanelet>> CommonRoadFactory2018b::createLanelets() {
	std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};

	pugi::xml_node commonRoad = doc->child("commonRoad");
	// get the number of lanelets
	size_t n = std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end());
	tempLaneletContainer.clear();
	tempLaneletContainer.reserve(n); // Already know the size --> Faster memory allocation

	/*
	 * all lanelets must be initialized first because they are referencing
	 * each other
	 */

	for (size_t i = 0; i < n; i++) {
        Lanelet newLanelet;

		// make_shared is faster than (new vehicularLanelet());
		std::shared_ptr<Lanelet> tempLanelet = std::make_shared<Lanelet>();
		tempLaneletContainer.emplace_back(tempLanelet);
	}

	size_t arrayIndex = 0;
	// set id of the lanelets
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
		 roadElements = roadElements.next_sibling()) {
		if (!(strcmp(roadElements.name(), "lanelet"))) {
			tempLaneletContainer[arrayIndex]->setId(roadElements.first_attribute().as_int());
			arrayIndex++;
		}
	}

	// get the other values of the lanelets
	arrayIndex = 0;
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
		 roadElements = roadElements.next_sibling()) {
		if (!(strcmp(roadElements.name(), "lanelet"))) {
			for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
				// set left bound
				if (!(strcmp(child.name(), "leftBound"))) {
					for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
						if (!(strcmp(points.name(), "point"))) {
							vertice newVertice;
							newVertice.x = points.child("x").text().as_double();
							newVertice.y = points.child("y").text().as_double();
							tempLaneletContainer[arrayIndex]->addLeftVertice(newVertice);
						}
					}
					continue;
				}
				// set right bound
				if (!(strcmp(child.name(), "rightBound"))) {
					for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling()) {
						if (!(strcmp(points.name(), "point"))) {
							vertice newVertice;
							newVertice.x = points.child("x").text().as_double();
							newVertice.y = points.child("y").text().as_double();
							tempLaneletContainer[arrayIndex]->addRightVertice(newVertice);
						}
					}
					continue;
				}
				// set successor lanelets
				if (!(strcmp(child.name(), "successor"))) {
					size_t successorId = child.first_attribute().as_int();
					for (size_t i = 0; i < n; i++) {
						if (tempLaneletContainer[i]->getId() == successorId) {
							tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i].get());
							break;
						}
					}
					continue;
				}
				// set predecessor lanelets
				if (!(strcmp(child.name(), "predecessor"))) {
					size_t predecessorId = child.first_attribute().as_int();
					for (size_t i = 0; i < n; i++) {
						if (tempLaneletContainer[i]->getId() == predecessorId) {
							tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i].get());
							break;
						}
					}
					continue;
				}
				// set left adjacent lanelets
				if (!(strcmp(child.name(), "adjacentLeft"))) {
					size_t adjacentId = child.attribute("ref").as_int();
					std::string dir = child.attribute("drivingDir").as_string();
					// std::cout << "string_dir: " << dir << std::endl;
					for (size_t i = 0; i < n; i++) {
						if (tempLaneletContainer[i]->getId() == adjacentId) {
							tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i].get(), dir);
							break;
						}
					}
					continue;
				}
				// set right adjacent lanelets
				if (!(strcmp(child.name(), "adjacentRight"))) {
					size_t adjacentId = child.attribute("ref").as_int();
					std::string dir = child.attribute("drivingDir").as_string();
					for (size_t i = 0; i < n; i++) {
						if (tempLaneletContainer[i]->getId() == adjacentId) {
							tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i].get(), dir);
							break;
						}
					}
					continue;
				}
				// set speed limit
//				if (!(strcmp(child.name(), "speedLimit"))) {
//					tempLaneletContainer[arrayIndex]->setSpeedLimit(child.text().as_double());
//					continue;
//				}
			}
			tempLaneletContainer[arrayIndex]->createCenterVertices();
			tempLaneletContainer[arrayIndex]->constructOuterPolygon();
			arrayIndex++;
		}
	}
	return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficSign>> CommonRoadFactory2018b::createTrafficSigns() {

    std::vector<std::shared_ptr<TrafficSign>> tempLaneletContainer{};
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficLight>> CommonRoadFactory2018b::createTrafficLights() {

    std::vector<std::shared_ptr<TrafficLight>> tempLaneletContainer{};
    return tempLaneletContainer;
}