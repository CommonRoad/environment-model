//
// Created by Sebastian Maierhofer on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "../regulatory_elements/traffic_light.h"
#include "../regulatory_elements/traffic_sign.h"
#include "../../auxiliaryDefs/structs.h"


typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::box<point_type> box;


class Lanelet {
    public:
        Lanelet() = default;
        Lanelet(int id,
                std::vector<vertice> leftBorder,
                std::vector<vertice> rightBorder,
                std::vector<LaneletType> type,
                std::vector<ObstacleType> oneWay = std::vector<ObstacleType>(),
                std::vector<ObstacleType> userBidirectional = std::vector<ObstacleType>());
        Lanelet(int id,
                std::vector<vertice> leftBorder,
                std::vector<vertice> rightBorder,
                std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                std::vector<std::shared_ptr<Lanelet>> successorLanelets,
                std::vector<LaneletType> laneletType,
                std::vector<ObstacleType> userOneWay,
                std::vector<ObstacleType> userBidirectional);
        Lanelet(const Lanelet &) = default;
        Lanelet &operator=(const Lanelet &) = default;
        Lanelet(Lanelet &&) = default;
        Lanelet &operator=(Lanelet &&) = default;
        virtual ~Lanelet() = default;

        /*
        *   adjacency struct with pointer to adjacent lanelet and information about its driving direction
        */
        struct adjacent {
            adjacent() : dir(DrivingDirection::invalid) {}
            std::shared_ptr<Lanelet> adj;
            DrivingDirection dir;
        };

        void setId(int num);
        void setLeftAdjacent(const std::shared_ptr<Lanelet>& left, DrivingDirection dir);
        void setRightAdjacent(const std::shared_ptr<Lanelet>& right, DrivingDirection dir);
        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices);
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices);
        void setAdjacentLeft(const adjacent &adjacentLeft);
        void setAdjacentRight(const adjacent &adjacentRight);
        void setLaneletType(const std::vector<LaneletType>& laneletType);
        void setUserOneWay(const std::vector<ObstacleType> &userOneWay);
        void setUserBidirectional(const std::vector<ObstacleType> &userBidirectional);

        void addCenterVertex(vertice center);
        void addLeftVertex(vertice left);
        void addRightVertex(vertice right);
        void addPredecessor(const std::shared_ptr<Lanelet>& pre);
        void addSuccessor(const std::shared_ptr<Lanelet>& suc);
        void addTrafficLight(const std::shared_ptr<TrafficLight>& light);
        void addTrafficSign(const std::shared_ptr<TrafficSign>& sign);

        [[nodiscard]] int getId() const;
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getPredecessors() const;
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getSuccessors() const;
        [[nodiscard]] const std::vector<vertice> &getCenterVertices() const;
        [[nodiscard]] const std::vector<vertice> &getLeftBorderVertices() const;
        [[nodiscard]] const std::vector<vertice> &getRightBorderVertices() const;
        [[nodiscard]] std::vector<std::shared_ptr<TrafficLight>> getTrafficLights() const;
        [[nodiscard]] std::vector<std::shared_ptr<TrafficSign>> getTrafficSigns() const;
        [[nodiscard]] const polygon_type &getOuterPolygon() const;
        [[nodiscard]] const box &getBoundingBox() const;
        [[nodiscard]] const std::vector<LaneletType> &getLaneletType() const;
        [[nodiscard]] const std::vector<ObstacleType> &getUserOneWay() const;
        [[nodiscard]] const std::vector<ObstacleType> &getUserBidirectional() const;
        [[nodiscard]] const adjacent &getAdjacentLeft() const;
        [[nodiscard]] const adjacent &getAdjacentRight() const;

        /**
        * Given a polygon, checks whether the polygon intersects with the lanelet
        *
        * @param polygon_shape boost polygon
        * @return boolean indicating whether lanelet is occupied
        */
        [[nodiscard]] bool applyIntersectionTesting(const polygon_type &polygon_shape) const;

        /**
        * Given a polygon, checks whether the polygon intersects with the lanelet given an intersection category
        *
        * @param polygon_shape boost polygon
        * @param intersection_flag specifies whether shape can be partially occupied by lanelet
         * or must be completely occupied
        * @return boolean indicating whether lanelet is occupied
        */
        [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, int intersection_flag) const;

        /**
        * Calculates center vertices as the arithmetic mean between the vertex on the left and right border
        */
        void createCenterVertices();

        /**
        * Constructs boost polygon representing shape of lanelet
        */
        void constructOuterPolygon();

        /**
        * Computes orientation of lanelet given a x- and y-position
        *
        * @param positionX x-position of point
        * @param positionY y-position of point
        * @return orientation in interval [-pi, pi]
        */
        double getOrientationAtPosition(double positionX, double positionY);


private:
        int id{};                                                   //**< unique ID of lanelet */
        std::vector<vertice> centerVertices;                        //**< vertices of center line of lanelet */
        std::vector<vertice> leftBorder;                            //**< vertices of left border */
        std::vector<vertice> rightBorder;                           //**< vertices of right border */
        std::vector<std::shared_ptr<Lanelet>> predecessorLanelets;  //**< list of pointers to predecessor lanelets */
        std::vector<std::shared_ptr<Lanelet>> successorLanelets;    //**< list of pointers to successor lanelets */
        adjacent adjacentLeft;                                      //**< pointer to left adjacent lanelet and info about its driving direction */
        adjacent adjacentRight;                                     //**< pointer to right adjacent lanelet and info about its driving direction */
        polygon_type outerPolygon;                                  //**< Boost polygon of the lanelet */
        box boundingBox{};                                          //**< Boost bounding box of the lanelet */
        std::vector<std::shared_ptr<TrafficLight>> trafficLights;   //**< list of pointers to traffic lights assigned to lanelet*/
        std::vector<std::shared_ptr<TrafficSign>> trafficSigns;     //**< list of pointers to traffic signs assigned to lanelet*/
        std::vector<LaneletType> laneletType;                       //**< list of relevant lanelet types*/
        std::vector<ObstacleType> userOneWay;                       //**< list of relevant allowed users one way*/
        std::vector<ObstacleType> userBidirectional;                //**< list of relevant allowed users bidirectional*/
};

#endif //ENVIRONMENT_MODEL_LANELET_H
