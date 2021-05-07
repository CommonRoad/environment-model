//
// Created by Sebastian Maierhofer on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_H
#define ENVIRONMENT_MODEL_LANELET_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "../../auxiliaryDefs/structs.h"
#include "../regulatoryElements/stop_line.h"
#include "../regulatoryElements/traffic_light.h"
#include "../regulatoryElements/traffic_sign.h"

namespace bg = boost::geometry;
using point_type = bg::model::d2::point_xy<double>;
using polygon_type = bg::model::polygon<point_type>;
using box = bg::model::box<point_type>;

/**
 * Class representing a lanelet.
 */
class Lanelet {
  public:
    /**
     * Default Constructor without parameters for a lanelet.
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
    Lanelet(int id, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder, std::vector<LaneletType> type,
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
    Lanelet(int id, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
            std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
            std::vector<std::shared_ptr<Lanelet>> successorLanelets, std::vector<LaneletType> laneletType,
            std::vector<ObstacleType> userOneWay, std::vector<ObstacleType> userBidirectional);

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
     * @param laneletId ID of lanelet.
     */
    void setId(int laneletId);

    /**
     * Setter for adjacent left lanelet.
     *
     * @param left pointer to left lanelet.
     * @param dir driving direction of adjacent left lanelet.
     */
    void setLeftAdjacent(const std::shared_ptr<Lanelet> &left, DrivingDirection dir);

    /**
     * Setter for adjacent right lanelet.
     *
     * @param right pointer to right lanelet.
     * @param dir driving direction of adjacent right lanelet.
     */
    void setRightAdjacent(const std::shared_ptr<Lanelet> &right, DrivingDirection dir);

    /**
     * Setter for left lanelet border vertices.
     *
     * @param leftBorderVertices Vector of vertices of left lanelet border.
     */
    void setLeftBorderVertices(const std::vector<vertex> &leftBorderVertices);

    /**
     * Setter for right lanelet border vertices.
     *
     * @param rightBorderVertices Vector of vertices of right lanelet border.
     */
    void setRightBorderVertices(const std::vector<vertex> &rightBorderVertices);

    /**
     * Setter for lanelet type.
     *
     * @param laneletType list of types classifying lanelet.
     */
    void setLaneletType(const std::vector<LaneletType> &laneletType);

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
    void setStopLine(const std::shared_ptr<StopLine> &sl);

    /**
     * Setter for left line marking.
     *
     * @param marking Left line marking.
     */
    void setLineMarkingLeft(LineMarking marking);

    /**
     * Setter for right line marking.
     *
     * @param marking Right line marking.
     */
    void setLineMarkingRight(LineMarking marking);

    /**
     * Appends a center vertex.
     *
     * @param center Vertex which should be appended.
     */
    void addCenterVertex(vertex center);

    /**
     * Appends a vertex to the left border.
     *
     * @param right Vertex which should be appended.
     */
    void addLeftVertex(vertex left);

    /**
     * Appends a vertex to the right border.
     *
     * @param right Vertex which should be appended.
     */
    void addRightVertex(vertex right);

    /**
     * Add a predecessor lanelet.
     *
     * @param pre Pointer to lanelet which should be added as predecessor.
     */
    void addPredecessor(const std::shared_ptr<Lanelet> &pre);

    /**
     * Add a successor lanelet.
     *
     * @param suc Pointer to lanelet which should be added as successor.
     */
    void addSuccessor(const std::shared_ptr<Lanelet> &suc);

    /**
     * Add a traffic light to lanelet.
     *
     * @param light Pointer to traffic light which should be added to lanelet.
     */
    void addTrafficLight(const std::shared_ptr<TrafficLight> &light);

    /**
     * Add a traffic sign to lanelet.
     *
     * @param sign Pointer to traffic sign which should be added to lanelet.
     */
    void addTrafficSign(const std::shared_ptr<TrafficSign> &sign);

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
    [[nodiscard]] const std::vector<vertex> &getCenterVertices() const;

    /**
     * Getter for left border vertices of lanelet.
     *
     * @return Vector containing vertices of left border.
     */
    [[nodiscard]] const std::vector<vertex> &getLeftBorderVertices() const;

    /**
     * Getter for right border vertices of lanelet.
     *
     * @return Vector containing vertices of right border.
     */
    [[nodiscard]] const std::vector<vertex> &getRightBorderVertices() const;

    /**
     * Getter for traffic lights referenced to lanelet.
     *
     * @return List of pointers to traffic lights.
     */
    [[nodiscard]] std::vector<std::shared_ptr<TrafficLight>> getTrafficLights() const;

    /**
     * Getter for traffic signs referenced to lanelet.
     *
     * @return List of pointers to traffic signs.
     */
    [[nodiscard]] std::vector<std::shared_ptr<TrafficSign>> getTrafficSigns() const;

    /**
     * Getter for polygon spanning the lanelet.
     *
     * @return Boost polygon.
     */
    [[nodiscard]] const polygon_type &getOuterPolygon() const;

    /**
     * Getter for bounding box of lanelet.
     *
     * @return Boost box.
     */
    [[nodiscard]] const box &getBoundingBox() const;

    /**
     * Getter for lanelet types.
     *
     * @return List of lanelet types.
     */
    [[nodiscard]] const std::vector<LaneletType> &getLaneletType() const;

    /**
     * Getter for road users one way.
     *
     * @return List of road users one way.
     */
    [[nodiscard]] const std::vector<ObstacleType> &getUserOneWay() const;

    /**
     * Getter for road users bidirectional.
     *
     * @return List of road users bidirectional.
     */
    [[nodiscard]] const std::vector<ObstacleType> &getUserBidirectional() const;

    /**
     * Getter for adjacent left lanelet.
     *
     * @return Adjacent left lanelet struct.
     */
    [[nodiscard]] const adjacent &getAdjacentLeft() const;

    /**
     * Getter for adjacent right lanelet.
     *
     * @return Adjacent right lanelet struct.
     */
    [[nodiscard]] const adjacent &getAdjacentRight() const;

    /**
     * Getter for stop line assigned to lanelet.
     *
     * @return Stop line.
     */
    [[nodiscard]] const std::shared_ptr<StopLine> &getStopLine() const;

    /**
     * Getter for left line marking of lanelet.
     *
     * @return Left line marking.
     */
    [[nodiscard]] LineMarking getLineMarkingLeft() const;

    /**
     * Getter for right line marking of lanelet.
     *
     * @return Right line marking.
     */
    [[nodiscard]] LineMarking getLineMarkingRight() const;

    /**
     * Given a polygon, checks whether the polygon intersects with the lanelet.
     *
     * @param polygon_shape boost polygon
     * @return boolean indicating whether lanelet is occupied
     */
    [[nodiscard]] bool applyIntersectionTesting(const polygon_type &polygon_shape) const;

    /**
     * Given a polygon, checks whether the polygon intersects with the lanelet given an intersection category.
     *
     * @param polygon_shape boost polygon
     * @param intersection_type specifies whether shape can be partially occupied by lanelet
     *  or must be completely occupied
     * @return boolean indicating whether lanelet is occupied
     */
    [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const;

    /**
     * Calculates center vertices as the arithmetic mean between the vertex on the left and right border.
     */
    void createCenterVertices();

    /**
     * Constructs boost polygon representing shape of lanelet based on left and right border vertices.
     */
    void constructOuterPolygon();

    /**
     * Computes orientation of lanelet given a x- and y-position of center vertices.
     *
     * @param positionX x-position of point
     * @param positionY y-position of point
     * @return orientation in interval [-pi, pi]
     */
    double getOrientationAtPosition(double positionX, double positionY);

    /**
     * Evaluates whether a lanelet has a specific lanelet type.
     *
     * @param laType Lanelet type which existence should be checked.
     * @return Boolean indicating whether the lanelet type exists.
     */
    bool hasLaneletType(LaneletType laType);

    /**
     * Adds a lanelet type to the lanelet.
     *
     * @param laType Lanelet type which should be added.
     */
    void addLaneletType(LaneletType laType);

  private:
    int id{};                                                  //**< unique ID of lanelet */
    std::vector<vertex> centerVertices;                        //**< vertices of center line of lanelet */
    std::vector<vertex> leftBorder;                            //**< vertices of left border */
    std::vector<vertex> rightBorder;                           //**< vertices of right border */
    std::vector<std::shared_ptr<Lanelet>> predecessorLanelets; //**< list of pointers to predecessor lanelets */
    std::vector<std::shared_ptr<Lanelet>> successorLanelets;   //**< list of pointers to successor lanelets */
    adjacent adjacentLeft;     //**< pointer to left adjacent lanelet and info about its driving direction */
    adjacent adjacentRight;    //**< pointer to right adjacent lanelet and info about its driving direction */
    polygon_type outerPolygon; //**< Boost polygon of the lanelet */
    box boundingBox{};         //**< Boost bounding box of the lanelet */
    std::vector<std::shared_ptr<TrafficLight>>
        trafficLights; //**< list of pointers to traffic lights assigned to lanelet*/
    std::vector<std::shared_ptr<TrafficSign>>
        trafficSigns;                            //**< list of pointers to traffic signs assigned to lanelet*/
    std::vector<LaneletType> laneletType;        //**< list of relevant lanelet types*/
    std::vector<ObstacleType> userOneWay;        //**< list of relevant allowed users one way*/
    std::vector<ObstacleType> userBidirectional; //**< list of relevant allowed users bidirectional*/
    std::shared_ptr<StopLine> stopLine;          //**< stopLine assigned to lanelet*/
    LineMarking lineMarkingLeft;                 //**< Line marking of left boundary*/
    LineMarking lineMarkingRight;                //**< Line marking of right boundary*/
};

#endif // ENVIRONMENT_MODEL_LANELET_H
