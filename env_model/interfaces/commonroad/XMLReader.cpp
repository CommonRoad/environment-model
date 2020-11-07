//
// Created by sebastian on 30.10.20.
//

#include "XMLReader.h"
#include "pugi_xml/pugixml.hpp"
#include "CommonRoadFactory2018b.h"
#include "commonroad_factory_2020a.h"

std::unique_ptr<CommonRoadFactory> createCommonRoadFactory(const std::string &xmlFile) {
    std::unique_ptr<pugi::xml_document> doc = std::make_unique<pugi::xml_document>();

    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File");

    const auto version = doc->child("commonRoad").attribute("commonRoadVersion").value();
    if (!strcmp(version, "2017a") || !strcmp(version, "2018b"))
        return std::make_unique<CommonRoadFactory2018b>(std::move(doc));
    else if (!strcmp(version, "2020a"))
        return std::make_unique<CommonRoadFactory2020a>(std::move(doc));
    else
        throw std::runtime_error("This CommonRoad version is not supported.");
}

std::vector<std::shared_ptr<Obstacle>> XMLReader::createObstacleFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createObstacles();
}

std::vector<std::shared_ptr<Lanelet>> XMLReader::createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> sign, std::vector<std::shared_ptr<TrafficLight>> light) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createLanelets(sign, light);
}

std::vector<std::shared_ptr<TrafficSign>> XMLReader::createTrafficSignFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficSigns();
}

std::vector<std::shared_ptr<TrafficLight>> XMLReader::createTrafficLightFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficLights();
}