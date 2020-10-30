//
// Created by sebastian on 30.10.20.
//

#ifndef ENV_MODEL_XMLREADER_H
#define ENV_MODEL_XMLREADER_H

#pragma once

#include "../../auxiliaryDefs/structs.h"
#include "../../road_network/lanelet/lanelet.h"
//#include "../world/obstacle/obstacle.h"
#include <cstddef>
#include <memory>
#include <vector>



namespace XMLReader {

// creates all obstacles objects (currently only vehicles) from the XML input
// std::vector<std::shared_ptr<obstacle>> createObstacleFromXML(const std::string &xmlFile,
//                                                              const obstacleParameters *param, const double
//                                                              &timeStamp);

//    std::vector<std::shared_ptr<obstacle>> createObstacleFromXML(const std::string &xmlFile, double timeStamp,
//                                                                 const obstacleParameters *param = NULL);

// std::vector<std::shared_ptr<obstacle>> createObstacleFromXML(const std::string &xmlFile, const double &timeStamp);

// creates all vehicular lanelet objects from the XML input
    std::vector<std::shared_ptr<Lanelet>> createLaneletFromXML(const std::string &xmlFile);

}; // namespace XMLReader




#endif //ENV_MODEL_XMLREADER_H
