//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_LANE_H
#define ENV_MODEL_LANE_H

#include "lanelet.h"
#include "geometry/curvilinear_coordinate_system.h"

typedef geometry::CurvilinearCoordinateSystem CurvilinearCoordinateSystem;

class Lane {
public:
    Lane(std::vector<std::shared_ptr<Lanelet>>  containedLanelets, Lanelet lanelet, CurvilinearCoordinateSystem ccs);
    ~Lane() = default;
    Lane(const Lane &other) = default;
    Lane& operator=(Lane const& other) { };
    Lane& operator=(Lane && other) = default;

    [[nodiscard]] bool checkIntersection(const polygon_type &intersecting, size_t intersection_flag) const;

    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getContainedLanelets() const;

    void setContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets);

    [[nodiscard]] Lanelet getLanelet() const;

    void setLanelet(const Lanelet &lanelet);

    [[nodiscard]] const CurvilinearCoordinateSystem &getCurvilinearCoordinateSystem() const;

    void setCurvilinearCoordinateSystem(const CurvilinearCoordinateSystem &curvilinearCoordinateSystem);

private:
        std::vector<std::shared_ptr<Lanelet>> containedLanelets;
        Lanelet lanelet;
        CurvilinearCoordinateSystem curvilinearCoordinateSystem;
};


#endif //ENV_MODEL_LANE_H
