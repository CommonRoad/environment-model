//
// Created by Sebastian Maierhofer on 09.11.20.
//

#include "lanelet_operations.h"

LaneletType matchLaneletTypeToString(const char *type){
    if (!(strcmp(type, "interstate")))
        return LaneletType::interstate;
    else if (!(strcmp(type, "urban")))
        return LaneletType::urban;
    else if (!(strcmp(type, "crosswalk")))
        return LaneletType::crosswalk;
    else if (!(strcmp(type, "busStop")))
        return LaneletType::busStop;
    else if (!(strcmp(type, "country")))
        return LaneletType::country;
    else if (!(strcmp(type, "highway")))
        return LaneletType::highway;
    else if (!(strcmp(type, "driveWay")))
        return LaneletType::driveWay;
    else if (!(strcmp(type, "mainCarriageWay")))
        return LaneletType::mainCarriageWay;
    else if (!(strcmp(type, "accessRamp")))
        return LaneletType::accessRamp;
    else if (!(strcmp(type, "exitRamp")))
        return LaneletType::exitRamp;
    else if (!(strcmp(type, "shoulder")))
        return LaneletType::shoulder;
    else if (!(strcmp(type, "bikeLane")))
        return LaneletType::bikeLane;
    else if (!(strcmp(type, "sidewalk")))
        return LaneletType::sidewalk;
    else if (!(strcmp(type, "busLane")))
        return LaneletType::busLane;
    else
        return LaneletType::unknown;
}

LineMarking matchLineMarkingToString(const char *type){
    if (!(strcmp(type, "solid")))
        return LineMarking::solid;
    else if (!(strcmp(type, "dashed")))
        return LineMarking::dashed;
    else if (!(strcmp(type, "broad_solid")))
        return LineMarking::broad_solid;
    else if (!(strcmp(type, "broad_dashed")))
        return LineMarking::broad_dashed;
    else if (!(strcmp(type, "no_marking")))
        return LineMarking::no_marking;
    else
        return LineMarking::unknown;
}

 void combineLaneletAndSuccessors(std::vector<Lanelet> *laneList, Lanelet *curLanelet, size_t k) {

     if (curLanelet == 0) {
         return;
     }
     // check for cyclic adjacencies
     lane *specificLane = &(*laneList)[k - 1];
     for (size_t i = 0; i < specificLane->getAssLanelets().size(); i++) {

         if (specificLane->getAssLanelets()[i]->getId() == curLanelet->getId()) {
             std::cout << "The cyclic adjacency of vehicularLanelet has been cut after one cycle!" << std::endl;
             return;
         }
     }

     std::vector<vertice> temp = specificLane->getLeftBorderVertices();
     for (size_t i = 1; i < curLanelet->getLeftBorderVertices().size(); i++) {
         temp.push_back(curLanelet->getLeftBorderVertices()[i]);
     }
     specificLane->setLeftBorderVertices(temp);

     temp = specificLane->getRightBorderVertices();
     for (size_t i = 1; i < curLanelet->getRightBorderVertices().size(); i++) {
         temp.push_back(curLanelet->getRightBorderVertices()[i]);
     }
     // temp.insert(temp.end(), curvehicularLanelet->getRightBorder().begin(),
     // curvehicularLanelet->getRightBorder().end());
     specificLane->setRightBorderVertices(temp);
     specificLane->addAssemblingLanelet(curLanelet);

     std::vector<double> tempSpeed = specificLane->getSpeedLimit();
     double compSpeed = tempSpeed.back();
     tempSpeed.pop_back();

     std::vector<double> curVector(curLanelet->getLeftBorderVertices().size() - 1, curLanelet->getSpeedLimit());
     tempSpeed.insert(tempSpeed.end(), curVector.begin(), curVector.end());
     specificLane->setSpeedLimit(tempSpeed);
     for (size_t m = 1; m < curLanelet->getCenterVertices().size(); m++) {
         specificLane->addCenterVertice(curLanelet->getCenterVertices()[m]);
     }

     // for all successor vehicularLanelets, copy this lane in the next rows of the struct
     for (int e = k + 1; e <= (int)curLanelet->getSuccessors().size() - 1 + (int)k; e++) {
         if ((int)(*laneList).size() < e) {
             lane aNewLane;
             aNewLane.copyLane((*laneList)[k - 1]);
             (*laneList).push_back(aNewLane);
         } else {
             (*laneList)[e - 1].copyLane((*laneList)[k - 1]);
         }
     }

     for (int n = 1; n <= (int)curLanelet->getSuccessors().size(); n++) {
         lane *specLane1 = &(*laneList)[k - 1];
         lane *specLane2 = &(*laneList)[n + k - 2];

         if ((specLane2->getAssLanelets().size() > specLane1->getAssLanelets().size()) ||
             (specLane2->getAssLanelets().back()->getId() !=
              specLane1->getAssLanelets()[specLane2->getAssLanelets().size() - 1]->getId()))

         {

             lane aLane;
             lane *specificLane = &(*laneList)[k - 1];

             std::vector<vehicularLanelet *> tempAssLanelets = aLane.getAssLanelets();
             tempAssLanelets.emplace_back(specificLane->getAssLanelets().front());
             aLane.setAssemblingLanelets(tempAssLanelets);

             aLane.setLeftBorderVertices(specificLane->getAssLanelets().front()->getLeftBorderVertices());
             aLane.setRightBorderVertices(specificLane->getAssLanelets().front()->getRightBorderVertices());
             std::vector<double> lim(specificLane->getAssLanelets().front()->getLeftBorderVertices().size(),
                                     specificLane->getAssLanelets().front()->getSpeedLimit());
             aLane.setSpeedLimit(lim);
             aLane.setCenterVertices(specificLane->getAssLanelets().front()->getCenterVertices());
             (*laneList).push_back(aLane);
             size_t p = 1;

             while ((*laneList).back().getAssLanelets().back()->getId() != curLanelet->getId()) {
                 (*laneList).back().addAssemblingLanelet((*laneList)[k - 1].getAssLanelets()[p]);
                 std::vector<vertice> temp = (*laneList).back().getLeftBorderVertices();
                 for (size_t i = 1; i < (*laneList)[k - 1].getAssLanelets()[p]->getLeftBorderVertices().size(); i++) {
                     temp.push_back((*laneList)[k - 1].getAssLanelets()[p]->getLeftBorderVertices()[i]);
                 }
                 (*laneList).back().setLeftBorderVertices(temp);
                 temp = (*laneList).back().getRightBorderVertices();
                 for (size_t i = 1; i < (*laneList)[k - 1].getAssLanelets()[p]->getRightBorderVertices().size(); i++)
                 {
                     temp.push_back((*laneList)[k - 1].getAssLanelets()[p]->getRightBorderVertices()[i]);
                 }
                 (*laneList).back().setRightBorderVertices(temp);
                 std::vector<double> tempSp = (*laneList).back().getSpeedLimit();
                 double speed = tempSp.back();
                 tempSp.pop_back();
                 if (speed > (*laneList)[k - 1].getAssLanelets()[p]->getSpeedLimit()) {
                     tempSp.push_back(speed);
                 } else {
                     tempSp.push_back((*laneList)[k - 1].getAssLanelets()[p]->getSpeedLimit());
                 }
                 std::vector<double> currVector((*laneList)[k - 1].getAssLanelets()[p]->getLeftBorderVertices().size()
                 -
                                                    1,
                                                (*laneList)[k - 1].getAssLanelets()[p]->getSpeedLimit());
                 tempSp.insert(tempSp.end(), currVector.begin(), currVector.end());
                 (*laneList).back().setSpeedLimit(tempSp);
                 for (size_t m = 1; m < (*laneList)[k - 1].getAssLanelets()[p]->getCenterVertices().size(); m++) {
                     (*laneList).back().addCenterVertice((*laneList)[k -
                     1].getAssLanelets()[p]->getCenterVertices()[m]);
                 }
                 p++;
             }
             combineLaneletAndSuccessors(laneList, curLanelet->getSuccessors()[n - 1], (*laneList).size());
         } else {
             combineLaneletAndSuccessors(laneList, curLanelet->getSuccessors()[n - 1], k + n - 1);
         }
     }
 }
