//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "stop_line.h"

const std::vector<vertex> &StopLine::getPoints() const { return points; }

void StopLine::setPoints(const std::vector<vertex> &po) { points = po; }

std::vector<std::shared_ptr<TrafficSign>> StopLine::getTrafficSigns() const { return trafficSign; }

void StopLine::setTrafficSigns(std::vector<std::shared_ptr<TrafficSign>> sign) { trafficSign = std::move(sign); }

std::vector<std::shared_ptr<TrafficLight>> StopLine::getTrafficLights() const { return trafficLight; }

void StopLine::setTrafficLights(std::vector<std::shared_ptr<TrafficLight>> light) { trafficLight = std::move(light); }

LineMarking StopLine::getLineMarking() const { return lineMarking; }

void StopLine::setLineMarking(LineMarking marking) { lineMarking = marking; }

void StopLine::addTrafficLight(const std::shared_ptr<TrafficLight> &light) { trafficLight.push_back(light); }

void StopLine::addTrafficSign(const std::shared_ptr<TrafficSign> &sign) { trafficSign.push_back(sign); }