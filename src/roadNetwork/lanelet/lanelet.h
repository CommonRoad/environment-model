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
#include "../regulatory_elements/stop_line.h"
#include "../../auxiliaryDefs/structs.h"

using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type =  boost::geometry::model::polygon<point_type>;
using box =  boost::geometry::model::box<point_type>;

/**
 * Class representing a lanelet.
 */
class Lanelet {
    public:
        /**
        * Default Constructor for a lanelet without parameters.
        */
        Lanelet() = default;

        /**
        * Constructor initializing borders, lanelet typ, and road users.
         * Creates automatically center vertices and polygon shape.
         *
         * @param leftBorder Vector of vertices of left lanelet border.
         * @param rightBorder Vector of vertices of right lanelet border.
         * @param type List of types classifying lanelet.
         * @param oneWay List of road users one way.
         * @param userBidirectional List of road users bidirectional.
        */
        Lanelet(int id,
                std::vector<vertice> leftBorder,
                std::vector<vertice> rightBorder,
                std::vector<LaneletType> type,
                std::vector<ObstacleType> oneWay = std::vector<ObstacleType>(),
                std::vector<ObstacleType> userBidirectional = std::vector<ObstacleType>());

        /**
         * Constructor initializing borders, lanelet typ, road users, predecessor lanelets, and successor lanelets.
         * Creates automatically center vertices and polygon shape.
         *
         * @param leftBorder Vector of vertices of left lanelet border.
         * @param rightBorder Vector of vertices of right lanelet border.
         * @param predecessorLanelets List of pointers to predecessor lanelets.
         * @param successorLanelets List of pointers to successor lanelets.
         * @param type List of types classifying lanelet.
         * @param oneWay List of road users one way.
         * @param userBidirectional List of road users bidirectional.
        */
        Lanelet(int id,
                std::vector<vertice> leftBorder,
                std::vector<vertice> rightBorder,
                std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                std::vector<std::shared_ptr<Lanelet>> successorLanelets,
                std::vector<LaneletType> laneletType,
                std::vector<ObstacleType> userOneWay,
                std::vector<ObstacleType> userBidirectional);

        /*
        *   Adjacency struct containing a pointer to the adjacent lanelet and information about its driving direction.
        */
        struct adjacent {
            adjacent() : dir(DrivingDirection::invalid) {}
            std::shared_ptr<Lanelet> adj;
            DrivingDirection dir;
        };

        /**
         * Setter for ID of lanelet.
         *
         * @param laneletId Lanelet ID.
         */
        void setId(int laneletId);

        /**
         * Setter for adjacent left lanelet.
         *
         * @param left pointer to left lanelet.
         * @param dir driving direction of adjacent left lanelet.
        */
        void setLeftAdjacent(const std::shared_ptr<Lanelet>& left, DrivingDirection dir);

        /**
         * Setter for adjacent right lanelet.
         *
         * @param right pointer to right lanelet.
         * @param dir driving direction of adjacent right lanelet.
        */
        void setRightAdjacent(const std::shared_ptr<Lanelet>& right, DrivingDirection dir);

        /**
         * Setter for left lanelet border vertices.
         *
         * @param leftBorderVertices Vector of vertices of left lanelet border.
        */
        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices);

        /**
         * Setter for right lanelet border vertices.
         *
         * @param rightBorderVertices Vector of vertices of right lanelet border.
        */
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices);

        /**
         * Setter for lanelet type.
         *
         * @param laneletType list of types classifying lanelet.
        */
        void setLaneletType(const std::vector<LaneletType>& laneletType);

        /**
         * Setter for road users one way.
         *
         * @param userOneWay List of road users one way.
        */
        void setUserOneWay(const std::vector<ObstacleType> &userOneWay);

        /**
         * Setter for road users bidirectional.
         *
         * @param userBidirectional List of road users bidirectional.
        */
        void setUserBidirectional(const std::vector<ObstacleType> &userBidirectional);

        /**
         * Setter for stop line.
         *
         * @param sl Stop line belonging to lanelet.
        */
        void setStopLine(const StopLine &sl);

        /**
         * Appends a center vertex.
         *
         * @param center Vertex which should be appended.
        */
        void addCenterVertex(vertice center);

        /**
         * Appends a vertex to the left border.
         *
         * @param right Vertex which should be appended.
        */
        void addLeftVertex(vertice left);

        /**
         * Appends a vertex to the right border.
         *
         * @param right Vertex which should be appended.
        */
        void addRightVertex(vertice right);

        /**
         * Add a predecessor lanelet.
         *
         * @param pre Pointer to lanelet which should be added as predecessor.
        */
        void addPredecessor(const std::shared_ptr<Lanelet>& pre);

        /**
         * Add a successor lanelet.
         *
         * @param suc Pointer to lanelet which should be added as successor.
        */
        void addSuccessor(const std::shared_ptr<Lanelet>& suc);

        /**
         * Add a traffic light to lanelet.
         *
         * @param light Pointer to traffic light which should be added to lanelet.
        */
        void addTrafficLight(const std::shared_ptr<TrafficLight>& light);

        /**
         * Add a traffic sign to lanelet.
         *
         * @param sign Pointer to traffic sign which should be added to lanelet.
        */
        void addTrafficSign(const std::shared_ptr<TrafficSign>& sign);

        /**
         * Getter for lanelet ID.
         *
         * @return Lanelet ID.
        */
        [[nodiscard]] int getId() const;

        /**
         * Getter for predecessor lanelets.
         *
         * @return List of pointers to predecessor lanelets.
        */
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getPredecessors() const;

        /**
         * Getter for successor lanelets.
         *
         * @return List of pointers to successor lanelets.
        */
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getSuccessors() const;

        /**
         * Getter for center vertices of lanelet.
         *
         * @return Vector containing center vertices.
        */
        [[nodiscard]] const std::vector<vertice> &getCenterVertices() const;

        /**
         * Getter for left border vertices of lanelet.
         *
         * @return Vector containing vertices of left border.
        */
        [[nodiscard]] const std::vector<vertice> &getLeftBorderVertices() const;

        /**
         * Getter for right border vertices of lanelet.
         *
         * @return Vector containing vertices of right border.
        */
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
        [[nodiscard]] const StopLine &getStopLine() const;

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
         * @param intersection_type specifies whether shape can be partially occupied by lanelet
         *  or must be completely occupied
         * @return boolean indicating whether lanelet is occupied
        */
        [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, int intersection_type) const;

        /**
         * Calculates center vertices as the arithmetic mean between the vertex on the left and right border
        */
        void createCenterVertices();

        /**
         * Constructs boost polygon representing shape of lanelet
        */
        void constructOuterPolygon();

        /**
         * Computes orientation of lanelet given x- and y-position
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
        StopLine stopLine;                                          //**< stopLine assigned to lanelet*/
};

#endif //ENVIRONMENT_MODEL_LANELET_H
