//
// Created by Sebastian Maierhofer on 30.10.20.
//

#ifndef ENV_MODEL_XML_READER_H
#define ENV_MODEL_XML_READER_H

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/interfaces/commonroad/pugi_xml/pugixml.hpp"

namespace XMLReader {
/**
 * Function for creating obstacles.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created obstacles.
 */
std::vector<std::shared_ptr<Obstacle>> createObstacleFromXML(const std::string &xmlFile);

/**
 * Function for creating lanelets.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights.
 * @return List of pointers to created lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> createLaneletFromXML(const std::string &xmlFile,
                                                           std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                                           std::vector<std::shared_ptr<TrafficLight>> trafficLights);

/**
 * Function for creating traffic signs.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created traffic signs.
 */
std::vector<std::shared_ptr<TrafficSign>> createTrafficSignFromXML(const std::string &xmlFile);

/**
 * Function for creating traffic lights.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created traffic lights.
 */
std::vector<std::shared_ptr<TrafficLight>> createTrafficLightFromXML(const std::string &xmlFile);

/**
 * Function for creating intersections.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @param lanelets List of pointers to lanelets.
 * @return List of pointers to created intersections.
 */
std::vector<std::shared_ptr<Intersection>>
createIntersectionFromXML(const std::string &xmlFile, const std::vector<std::shared_ptr<Lanelet>> &lanelets);

/**
 * Extracts an initial state from a XML node element.
 *
 * @param child XML node element.
 * @return Pointer to initial state.
 */
std::shared_ptr<State> extractInitialState(const pugi::xml_node &child);

/**
 * Extracts an state from a XML node element.
 *
 * @param child XML node element.
 * @return Pointer to state.
 */
std::shared_ptr<State> extractState(const pugi::xml_node &states);

/**
 * Creates a dynamic obstacle from a XML node element and adds the obstacle to provided vector.
 *
 * @param obstacleList Vector in which new dynamic obstacle are stored.
 * @param roadElements XML node element.
 */
void createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList, const pugi::xml_node &roadElements);

/**
 * Creates a static obstacle from a XML node element and adds the obstacle to provided vector.
 *
 * @param obstacleList Vector in which new static obstacle are stored.
 * @param roadElements XML node element.
 */
void extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList, const pugi::xml_node &roadElements);

/**
 * Initializes all lanelets without information using default constructor.
 *
 * @param tempLaneletContainer Vector in which new lanelets are stored.
 * @param commonRoad XML node element.
 * @return Number of created lanelets.
 */
int initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, const pugi::xml_node &commonRoad);

/**
 * Extracts lanelet boundary for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which boundary should be extracted.
 * @param child XML node element.
 * @param side String which defines whether left or right lanelet boundary should be extracted.
 */
void extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, int arrayIndex,
                            const pugi::xml_node &child, const char *side);

/**
 * Extracts successor or predecessor lanelet for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which successor/predecessor should be extracted.
 * @param child XML node element.
 * @param type String which defines whether successor or predecessor lanelet should be extracted.
 */
void extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, int arrayIndex,
                          const pugi::xml_node &child, const char *type);

/**
 * Extracts left or right adjacent lanelet for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which adjacent lanelet should be extracted.
 * @param child XML node element.
 * @param type String which defines whether left or right adjacent lanelet should be extracted.
 */
void extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, int arrayIndex,
                             const pugi::xml_node &child, const char *type);
} // namespace XMLReader

#endif // ENV_MODEL_XML_READER_H
