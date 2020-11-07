//
// Created by sebastian on 01.11.20.
//

#include "stop_line.h"

#include <utility>

const std::vector<vertice> &StopLine::getPoints() const {return points;}

void StopLine::setPoints(const std::vector<vertice> &po) {points = po;}

std::shared_ptr<TrafficSign> StopLine::getTrafficSign() const {return trafficSign;}

void StopLine::setTrafficSign(std::shared_ptr<TrafficSign> sign) {trafficSign = std::move(sign);}

std::shared_ptr<TrafficLight> StopLine::getTrafficLight() const {return trafficLight;}

void StopLine::setTrafficLight(std::shared_ptr<TrafficLight> light) {trafficLight = std::move(light);}

LineMarking StopLine::getLineMarking() const {return lineMarking;}

void StopLine::setLineMarking(LineMarking marking) {lineMarking = marking;}
