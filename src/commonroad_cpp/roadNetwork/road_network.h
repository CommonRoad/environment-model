//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_ROAD_NETWORK_H
#define ENV_MODEL_ROAD_NETWORK_H

#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using point = bg::model::point<float, 2, bg::cs::cartesian>;
using value = std::pair<box, unsigned>;

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
        std::vector<std::shared_ptr<Intersection>> inters = std::vector<std::shared_ptr<Intersection>>{},
        std::vector<std::shared_ptr<TrafficSign>> signs = std::vector<std::shared_ptr<TrafficSign>>{},
        std::vector<std::shared_ptr<TrafficLight>> lights = std::vector<std::shared_ptr<TrafficLight>>{});

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

    /**
     * Computes and sets dynamic intersection labels of incomings.
     */
    void setDynamicIntersectionLabels();

    std::shared_ptr<Incoming> incomingOfLanelet(const std::shared_ptr<Lanelet> &lanelet);

    /**
     * Matches a string to Country enum.
     *
     * @param name Name of country.
     * @return Country enum.
     */
    static SupportedTrafficSignCountry matchStringToCountry(const std::string &name);

  private:
    std::vector<std::shared_ptr<Lanelet>> laneletNetwork;     //**< set of lanelets contained in road network */
    SupportedTrafficSignCountry country;                      //**< country where road network is located */
    std::vector<std::shared_ptr<Intersection>> intersections; //**< set of intersections contained in road network */
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;   //**< set of traffic signs contained in road network */
    std::vector<std::shared_ptr<TrafficLight>> trafficLights; //**< set of traffic lights contained in road network */
    std::vector<std::shared_ptr<Lane>> lanes; //**< set of interstate-based lanes contained in road network */
    bgi::rtree<value, bgi::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/

    /**
     * Given a set of lanelets, creates the corresponding interstate-based lanes
     *
     * @param laneletNetwork set of lanelets
     */
    void createLanes(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

    /**
     * Given a lanelet, extracts the classifying lanelet type, e.g., for interstates, main carriageway, access/exit
     * ramp, or shoulder
     *
     * @param la lanelet for which classifying lanelet type should be extracted
     * @return classifying lanelet type
     */
    static LaneletType extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la);

    static std::vector<std::shared_ptr<Lanelet>>
    extractOutgoingsFromIncoming(const LaneletType &intersectionLaneletType,
                                 const std::vector<std::shared_ptr<Lanelet>> &incomingSuccessors);
};

#endif // ENV_MODEL_ROAD_NETWORK_H