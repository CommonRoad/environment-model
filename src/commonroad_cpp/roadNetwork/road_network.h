//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/index/parameters.hpp>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/types.h>

using value = std::pair<box, unsigned>;

class Lanelet;
class Lane;
class Incoming;
class TrafficLight;
class TrafficSign;
class Intersection;

/**
 * Class representing a road network.
 */
class RoadNetwork {
  public:
    /**
     * Constructor for RoadNetwork. Takes a lanelet network and automatically generates lanes out of them.
     * Additionally, creates and Rtree from the lanelets for faster access of lanelets and faster
     * occupancy computations of obstacles.
     *
     * @param list of pointers to lanelets
     */
    explicit RoadNetwork(
        const std::vector<std::shared_ptr<Lanelet>> &network,
        SupportedTrafficSignCountry cou = SupportedTrafficSignCountry::ZAMUNDA,
        std::vector<std::shared_ptr<TrafficSign>> signs = std::vector<std::shared_ptr<TrafficSign>>{},
        std::vector<std::shared_ptr<TrafficLight>> lights = std::vector<std::shared_ptr<TrafficLight>>{},
        std::vector<std::shared_ptr<Intersection>> inters = std::vector<std::shared_ptr<Intersection>>{});

    /**
     * Getter for lanelet network.
     *
     * @return list of pointers to lanelets
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;

    /**
     * Getter for lanes.
     *
     * @return list of pointers to lanes
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lane>> getLanes();

    /**
     * Getter for intersections.
     *
     * @return list of pointers to intersections.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Intersection>> &getIntersections() const;

    /**
     * Getter for country.
     *
     * @return Country where road network is located.
     */
    [[nodiscard]] SupportedTrafficSignCountry getCountry() const;

    /**
     * Given a polygon shape, finds the list of lanelets within the road network which intersect with the shape.
     *
     * @param polygonShape boost polygon
     * @return list of lanelet pointers
     */
    std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(const polygon_type &polygonShape);

    /**
     * Given a position, finds the list of lanelets within the road network which contain the point.
     *
     * @param xPos x-coordinate of point
     * @param yPos y-coordinate of point
     * @return list of lanelet pointers
     */
    std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(double xPos, double yPos);

    /**
     * Returns the lanelet which corresponds to a given lanelet ID.
     *
     * @param id lanelet ID
     * @return pointer to lanelet
     */
    std::shared_ptr<Lanelet> findLaneletById(size_t id);

    /**
     * Given a set of lanes and a polygon shape, finds a single lane which intersect with the shape (interstate)
     *
     * @param possibleLanes set of lanes
     * @param polygonShape boost polygon
     * @return list of lane pointers
     */
    static std::shared_ptr<Lane> findLaneByShape(const std::vector<std::shared_ptr<Lane>> &possibleLanes,
                                                 const polygon_type &polygonShape);

    std::shared_ptr<Incoming> incomingOfLanelet(const std::shared_ptr<Lanelet> &lanelet);

    /**
     * Matches a string to Country enum.
     *
     * @param name Name of country.
     * @return Country enum.
     */
    static SupportedTrafficSignCountry matchStringToCountry(const std::string &name);

    std::string extractTrafficSignIDForCountry(TrafficSignTypes type);

  private:
    std::vector<std::shared_ptr<Lanelet>> laneletNetwork;     //**< set of lanelets contained in road network */
    SupportedTrafficSignCountry country;                      //**< country where road network is located */
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;   //**< set of traffic signs contained in road network */
    std::vector<std::shared_ptr<TrafficLight>> trafficLights; //**< set of traffic lights contained in road network */
    std::vector<std::shared_ptr<Intersection>> intersections; //**< set of intersections contained in road network */
    std::vector<std::shared_ptr<Lane>> lanes; //**< set of interstate-based lanes contained in road network */
    boost::geometry::index::rtree<value, boost::geometry::index::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/
    const std::unordered_map<TrafficSignTypes, std::string> *trafficSignIDLookupTable; //**< mapping of traffic signs*/

    //**< interpreter for certain traffic signs*

    /**
     * Given a set of lanelets, creates the corresponding interstate-based lanes
     *
     * @param laneletNetwork set of lanelets
     */
    void createLanes(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);
};
