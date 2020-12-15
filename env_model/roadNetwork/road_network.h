//
// Created by sebastian on 08.11.20.
//

#ifndef ENV_MODEL_ROAD_NETWORK_H
#define ENV_MODEL_ROAD_NETWORK_H

#include "lanelet/lane.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using point =  bg::model::point<float, 2, bg::cs::cartesian>;
using value = std::pair<box, unsigned>;
using box_road =  bg::model::box<point>; //TODO: check whether this box and the box defined in lanelet.h can be combined

class RoadNetwork {
    public:
        explicit RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

        [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;
        std::vector<std::shared_ptr<Lane>> getLanes();

        /**
        * Given a polygon shape, finds the list of lanelets within the road network which intersect with the shape
        *
        * @param polygonShape boost polygon
        * @return list of lanelet pointers
        */
        std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(const polygon_type &polygonShape);

        /**
        * Given a position, finds the list of lanelets within the road network which contain the point
        *
        * @param xPos x-coordinate of point
        * @param yPos y-coordinate of point
        * @return list of lanelet pointers
        */
        std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(double xPos, double yPos);

        /**
        * Returns the lanelet which corresponds to a given lanelet ID
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
        static std::shared_ptr<Lane> findLaneByShape(const std::vector<std::shared_ptr<Lane>>& possibleLanes,
                                                     const polygon_type &polygonShape);

    private:
        std::vector<std::shared_ptr<Lanelet>> laneletNetwork;   //**< set of lanelets contained in road network */
        std::vector<std::shared_ptr<Lane>> lanes;               //**< set of interstate-based lanes contained in road network */
        bgi::rtree< value, bgi::quadratic<16>> rtree;           //**< rtree defined by lanelets of road network for faster occupancy calculation*/

        /**
        * Given a set of lanelets, creates the corresponding interstate-based lanes
        *
        * @param laneletNetwork set of lanelets
        */
        void createLanes(const std::vector<std::shared_ptr<Lanelet>>& laneletNetwork);

        /**
        * Given a lanelet, extracts the classifying lanelet type, e.g., for interstates, main carriageway, access/exit ramp, or shoulder
        *
        * @param la lanelet for which classifying lanelet type should be extracted
        * @return classifying lanelet type
        */
        static LaneletType extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la);
};

#endif //ENV_MODEL_ROAD_NETWORK_H
