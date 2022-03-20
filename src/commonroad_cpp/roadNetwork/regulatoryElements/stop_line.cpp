//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "stop_line.h"

#include <utility>

const std::vector<vertex> &StopLine::getPoints() const { return points; }

void StopLine::setPoints(const std::vector<vertex> &pos) { points = pos; }

std::vector<std::shared_ptr<TrafficSign>> StopLine::getTrafficSigns() const { return trafficSigns; }

void StopLine::setTrafficSigns(std::vector<std::shared_ptr<TrafficSign>> sign) { trafficSigns = std::move(sign); }

std::vector<std::shared_ptr<TrafficLight>> StopLine::getTrafficLights() const { return trafficLights; }

void StopLine::setTrafficLights(std::vector<std::shared_ptr<TrafficLight>> light) { trafficLights = std::move(light); }

LineMarking StopLine::getLineMarking() const { return lineMarking; }

void StopLine::setLineMarking(LineMarking marking) { lineMarking = marking; }

void StopLine::addTrafficLight(const std::shared_ptr<TrafficLight> &light) { trafficLights.push_back(light); }

void StopLine::addTrafficSign(const std::shared_ptr<TrafficSign> &sign) { trafficSigns.push_back(sign); }

StopLine::StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficSign>> traffic_sign,
                   std::vector<std::shared_ptr<TrafficLight>> traffic_light, LineMarking line_marking)
    : points(std::move(points)), trafficSigns(std::move(traffic_sign)), trafficLights(std::move(traffic_light)),
      lineMarking(line_marking) {}

StopLine::StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficSign>> traffic_sign,
                   LineMarking line_marking)
    : points(std::move(points)), trafficSigns(std::move(traffic_sign)), lineMarking(line_marking) {}

StopLine::StopLine(std::vector<vertex> points, std::vector<std::shared_ptr<TrafficLight>> traffic_light,
                   LineMarking line_marking)
    : points(std::move(points)), trafficLights(std::move(traffic_light)), lineMarking(line_marking) {}
