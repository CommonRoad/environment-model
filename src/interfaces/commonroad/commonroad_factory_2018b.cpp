//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include <obstacle/obstacle_operations.h>
#include "commonroad_factory_2018b.h"
#include "xml_reader.h"

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2018b::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "obstacle"))) {
            if (!(strcmp(roadElements.first_child().text().as_string(), "dynamic"))){
                XMLReader::createDynamicObstacle(obstacleList, roadElements);
            }
            else if (!(strcmp(roadElements.first_child().text().as_string(), "static"))) {
                XMLReader::extractStaticObstacle(obstacleList, roadElements);
            }
        }
    }
    return obstacleList;
}

std::vector<std::shared_ptr<Lanelet>> CommonRoadFactory2018b::createLanelets(
        std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
        std::vector<std::shared_ptr<TrafficLight>> trafficLights) {

    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");
    XMLReader::initializeLanelets(tempLaneletContainer, commonRoad);

    // get the other values of the lanelets
    int arrayIndex { 0 };
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements;
         roadElements = roadElements.next_sibling()) {
        if (!(strcmp(roadElements.name(), "lanelet"))) {
            for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling()) {
                // set left bound
                if (!(strcmp(child.name(), "leftBound"))) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "leftBound");
                    continue;
                }
                // set right bound
                if (!(strcmp(child.name(), "rightBound"))) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "rightBound");
                    continue;
                }
                // set successor lanelets
                if (!(strcmp(child.name(), "successor"))) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "successor");
                    continue;
                }
                // set predecessor lanelets
                if (!(strcmp(child.name(), "predecessor"))) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "predecessor");
                    continue;
                }
                // set left adjacent lanelets
                if (!(strcmp(child.name(), "adjacentLeft"))) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentLeft");
                    continue;
                }
                // set right adjacent lanelets
                if (!(strcmp(child.name(), "adjacentRight"))) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child,"adjacentRight");
                    continue;
                }
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

std::vector<std::shared_ptr<Intersection>> CommonRoadFactory2018b::createIntersections(
        const std::vector<std::shared_ptr<Lanelet>>& lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempLaneletContainer{};
    return tempLaneletContainer;
}
