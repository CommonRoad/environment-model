//
// Created by sebastian on 07.05.22.
//

#include "lanelet_graph.h"
#include "iostream"
#include <algorithm>
#include <queue>
#include <vector>

void LaneletGraph::printPath(std::vector<size_t> &path) {
    size_t size = path.size();
    for (size_t i = 0; i < size; i++)
        std::cout << path[i] << " ";
    std::cout << std::endl;
}

// utility function to check if current
// vertex is already present in path
bool LaneletGraph::isNotVisited(size_t node, std::vector<size_t> &path) {
    for (const auto &elem : path)
        if (elem == node)
            return false;
    return true;
}

bool LaneletGraph::exists(const std::vector<std::vector<size_t>> &paths, const std::vector<size_t> &newPath) {
    for (const auto &path : paths) {
        if (newPath.size() != path.size())
            continue;
        bool identical{true};
        for (size_t idx{0}; idx < newPath.size(); ++idx)
            if (newPath.at(idx) != path.at(idx)) {
                identical = false;
                break;
            }
        if (identical)
            return true;
    }
    return false;
}

std::vector<std::vector<size_t>> LaneletGraph::findPaths(size_t src, size_t dst, bool considerAdjacency) {
    if (considerAdjacency)
        return findPaths(src, dst, graphAdjSuc);
    else
        return findPaths(src, dst, graphSuc);
}

std::vector<std::vector<size_t>> LaneletGraph::findPaths(size_t src, size_t dst,
                                                         const std::map<size_t, std::vector<size_t>> &graph) {
    // create a queue which stores the paths
    std::vector<std::vector<size_t>> paths;
    std::queue<std::vector<size_t>> que;

    // path vector to store the current path
    std::vector<size_t> path;
    path.push_back(src);
    que.push(path);
    while (!que.empty()) {
        path = que.front();
        que.pop();
        size_t last = path[path.size() - 1];

        // if last vertex is the desired destination
        // then print the path
        if (last == dst and !exists(paths, path)) {
            paths.push_back(path);
        }

        // traverse to all the nodes connected to
        // current vertex and push new path to queue
        for (size_t idx = 0; idx < graph.at(last).size(); idx++) {
            if (isNotVisited(graph.at(last)[idx], path)) {
                std::vector<size_t> newPath(path);
                newPath.push_back(graph.at(last)[idx]);
                que.push(newPath);
            }
        }
    }
    return paths;
}
LaneletGraph::LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    for (const auto &let : lanelets) {
        if (graphAdjSuc.find(let->getId()) == graphAdjSuc.end())
            graphAdjSuc[let->getId()] = {};
        if (graphSuc.find(let->getId()) == graphSuc.end())
            graphSuc[let->getId()] = {};
        if (let->getAdjacentLeft().adj != nullptr and let->getAdjacentLeft().dir == DrivingDirection::same and
            !std::any_of(graphAdjSuc[let->getId()].begin(), graphAdjSuc[let->getId()].end(),
                         [let](size_t lid) { return lid == let->getAdjacentLeft().adj->getId(); })) {
            graphAdjSuc[let->getId()].push_back(let->getAdjacentLeft().adj->getId());
            graphAdjSuc[let->getAdjacentLeft().adj->getId()].push_back(let->getId());
        }
        if (let->getAdjacentRight().adj != nullptr and let->getAdjacentRight().dir == DrivingDirection::same and
            !std::any_of(graphAdjSuc[let->getId()].begin(), graphAdjSuc[let->getId()].end(),
                         [let](size_t lid) { return lid == let->getAdjacentRight().adj->getId(); })) {
            graphAdjSuc[let->getId()].push_back(let->getAdjacentRight().adj->getId());
            graphAdjSuc[let->getAdjacentRight().adj->getId()].push_back(let->getId());
        }
        for (const auto &suc : let->getSuccessors()) {
            graphAdjSuc[let->getId()].push_back(suc->getId());
            graphSuc[let->getId()].push_back(suc->getId());
        }
    }
}
