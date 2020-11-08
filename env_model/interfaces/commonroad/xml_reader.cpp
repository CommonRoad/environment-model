//
// Created by Sebastian Maierhofer on 30.10.20.
//

#include "xml_reader.h"
#include <utility>
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

std::vector<std::shared_ptr<Lanelet>> XMLReader::createLaneletFromXML(
        const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
        std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createLanelets(std::move(trafficSigns), std::move(trafficLights));
}

std::vector<std::shared_ptr<TrafficSign>> XMLReader::createTrafficSignFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficSigns();
}

std::vector<std::shared_ptr<TrafficLight>> XMLReader::createTrafficLightFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficLights();
}

std::vector<std::shared_ptr<Intersection>> XMLReader::createIntersectionFromXML(
        const std::string &xmlFile, const std::vector<std::shared_ptr<Lanelet>>& lanelets) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createIntersections(lanelets);
}

State XMLReader::extractInitialState(const pugi::xml_node &child)  {
    pugi::xml_node states = child;
    State initialState;
    initialState.setTimeStep(0);
    initialState.setXPosition(
            states.child("position").child("point").child("x").text().as_double());
    initialState.setYPosition(
            states.child("position").child("point").child("y").text().as_double());
    initialState.setOrientation(states.child("orientation").child("exact").text().as_double());
    initialState.setVelocity(states.child("velocity").child("exact").text().as_double());
    initialState.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return initialState;
}

State XMLReader::extractState(const pugi::xml_node &states) {
    State st;
    st.setTimeStep(states.child("time").child("exact").text().as_int());
    st.setXPosition(states.child("position").child("point").child("x").text().as_double());
    st.setYPosition(states.child("position").child("point").child("y").text().as_double());
    st.setOrientation(states.child("orientation").child("exact").text().as_double());
    st.setVelocity(states.child("velocity").child("exact").text().as_double());
    st.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return st;
}