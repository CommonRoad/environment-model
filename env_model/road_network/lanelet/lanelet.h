//
// Created by Sebastian Maierhofer on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H


#include "../regulatory_elements/traffic_light.h"
#include "../regulatory_elements/traffic_sign.h"
#include "../../auxiliaryDefs/structs.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::box<point_type> box;


class Lanelet {
    public:
        Lanelet();                                     // default constructor
        Lanelet(const Lanelet &) = default;            // copy constructor
        Lanelet(size_t id, std::vector<vertice> centerVertices, std::vector<vertice> leftBorder,
                std::vector<vertice> rightBorder, std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                std::vector<std::shared_ptr<Lanelet>> successorLanelets,
                std::vector<LaneletType> laneletType, std::vector<ObstacleType> userOneWay,
                std::vector<ObstacleType> userBidirectional);

        Lanelet &operator=(const Lanelet &) = default; // copy assignment
        Lanelet(Lanelet &&) = default;


    // move constructor
        Lanelet &operator=(Lanelet &&) = default;      // move assignment
        virtual ~Lanelet() = default;                  // virtual destructor

        /*
         * setter functions
         */
        void setId(size_t num);
        void addLeftVertice(vertice left);
        void addRightVertice(vertice right);
        void addCenterVertice(vertice center);
        void addPredecessor(const std::shared_ptr<Lanelet>& pre);
        void addSuccessor(const std::shared_ptr<Lanelet>& suc);
        void setLeftAdjacent(Lanelet *left, const std::string& dir);
        void setRightAdjacent(Lanelet *right, const std::string& dir);
        void setLeftBorderVertices(const std::vector<vertice> &leftBorderVertices);
        void setRightBorderVertices(const std::vector<vertice> &rightBorderVertices);
        void setCenterVertices(const std::vector<vertice> &center);
        void addTrafficLight(const std::shared_ptr<TrafficLight>& light);
        void addTrafficSign(const std::shared_ptr<TrafficSign>& sign);
        void setOuterPolygon(const polygon_type &outerPolygon);
        void setBoundingBox(const box &boundingBox);
        void setLaneletType(const std::vector<LaneletType>& laneletType);
        void setUserOneWay(const std::vector<ObstacleType> &userOneWay);
        void setUserBidirectional(const std::vector<ObstacleType> &userBidirectional);

        // Takes rvalue and moves the data
        void moveLeftBorder(std::vector<vertice> &&leftBorderVertices);
        void moveRightBorder(std::vector<vertice> &&rightBorderVertices);
        void moveCenterVertices(std::vector<vertice> &&center);

        /*
         * getter functions
         */
        [[nodiscard]] size_t getId() const;
        [[nodiscard]] std::vector<vertice> getLeftBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getRightBorderVerticesDirect() const;
        [[nodiscard]] std::vector<vertice> getCenterVerticesDirect() const;
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getPredecessors() const;
        [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getSuccessors() const;
        [[nodiscard]] const std::vector<vertice> &getCenterVertices() const;
        [[nodiscard]] const std::vector<vertice> &getLeftBorderVertices() const;
        [[nodiscard]] const std::vector<vertice> &getRightBorderVertices() const;
        [[nodiscard]] std::vector<std::shared_ptr<TrafficLight>> getTrafficLight() const;
        [[nodiscard]] std::vector<std::shared_ptr<TrafficSign>> getTrafficSigns() const;
        [[nodiscard]] const polygon_type &getOuterPolygon() const;
        [[nodiscard]] const box &getBoundingBox() const;
        [[nodiscard]] const std::vector<LaneletType> &getLaneletType() const;
        [[nodiscard]] const std::vector<ObstacleType> &getUserOneWay() const;
        [[nodiscard]] const std::vector<ObstacleType> &getUserBidirectional() const;
        [[nodiscard]] bool applyIntersectionTesting(const polygon_type &intersection) const;
        [[nodiscard]] bool checkIntersection(const polygon_type &intersecting, size_t intersection_flag) const;

        void createCenterVertices();
        void constructOuterPolygon(); // construct outer shape from borders
        double getOrientationAtPosition(double positionX, double positionY);


    struct adjacent {
        std::vector<Lanelet *> adj;
        std::string dir;
    };

    private:
        size_t id{};                                          // unique ID of lanelet
        std::vector<vertice> centerVertices;                // vertices of center line of lanelet
        std::vector<vertice> leftBorder;                    // vertices of left border
        std::vector<vertice> rightBorder;                   // vertices of right border
        std::vector<std::shared_ptr<Lanelet>> predecessorLanelets;         // previous lanelets
        std::vector<std::shared_ptr<Lanelet>> successorLanelets; // longitudinally adjacent lanelets
        adjacent adjacentLeft;                              // left adjacent lanelet with driving tag
        adjacent adjacentRight;                             // right adjacent lanelet with driving tag
        polygon_type outerPolygon;
        box boundingBox{};                                  // Boost bounding box of the lanelet
        std::vector<std::shared_ptr<TrafficLight>> trafficLights;                  // traffic light assigned to lanelet
        std::vector<std::shared_ptr<TrafficSign>> trafficSigns;            // traffic signs assigned to lanelet
        std::vector<LaneletType> laneletType;
        std::vector<ObstacleType> userOneWay;
        std::vector<ObstacleType> userBidirectional;
};


#endif //ENVIRONMENT_MODEL_LANELET_H
