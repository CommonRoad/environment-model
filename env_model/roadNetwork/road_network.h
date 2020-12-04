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

template <typename First, typename Second>
struct pair_maker
{
    typedef std::pair<First, Second> result_type;
    template<typename T>
    inline result_type operator()(T const& v) const
    {
        return result_type(v.value(), v.index());
    }
};

class RoadNetwork {
    public:
        explicit RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);

        [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;
        std::vector<std::shared_ptr<Lane>> getLanes();

        void setLaneletNetwork(const std::vector<std::shared_ptr<Lanelet>> &laneletNetwork);
        void setLanes(std::vector<std::shared_ptr<Lane>> lanes);

        /**
        * Given a shape, finds the list of lanelets within the road network which intersect with the shape
        *
        * @param polygonShape boost polygon
        * @return list of shared lanelet pointers
        */
        std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(const polygon_type &polygonShape);
        std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(double xPos, double yPos);
        std::shared_ptr<Lanelet> findLaneletById(size_t id);

        static std::shared_ptr<Lane> findLaneByShape(std::vector<std::shared_ptr<Lane>> possibleLanes, const polygon_type &polygonShape);

    private:
        typedef bg::model::point<float, 2, bg::cs::cartesian> point;
        typedef bg::model::box<point> box;
        typedef bg::model::polygon<point, false, false> polygon; // ccw, open polygon
        typedef std::pair<box, unsigned> value;

        void createLanes(const std::vector<std::shared_ptr<Lanelet>>& laneletNetwork);
        std::vector<std::shared_ptr<Lanelet>> laneletNetwork;
        std::vector<std::shared_ptr<Lane>> lanes;
        bgi::rtree< value, bgi::quadratic<16>> rtree;

        static LaneletType extractClassifyingLaneletType(const std::shared_ptr<Lanelet> &la);
};


#endif //ENV_MODEL_ROAD_NETWORK_H
