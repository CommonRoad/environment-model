//
// Created by sebastian on 07.05.22.
//

#include "lanelet_graph.h"
#include <vector>

std::vector<size_t> LaneletGraph::findPaths(size_t src, size_t dst, bool considerAdjacency) {
    std::vector<size_t> path;
    std::optional<std::pair<std::vector<std::ptrdiff_t>, std::ptrdiff_t>> result;
    if (considerAdjacency) {
        dijkstra<size_t, size_t> searcher{graphAdjSuc, static_cast<ptrdiff_t>(verticesAdjSuc.at(src))};
        result = searcher.search_path(static_cast<ptrdiff_t>(verticesAdjSuc.at(dst)));
        for (const auto &res : result->first)
            path.push_back(verticesAdjSucRes.at(static_cast<const unsigned long>(res)));
    } else {
        dijkstra<size_t, size_t> searcher{graphSuc, static_cast<ptrdiff_t>(verticesAdjSuc.at(src))};
        result = searcher.search_path(static_cast<ptrdiff_t>(verticesAdjSuc.at(dst)));
        for (const auto &res : result->first)
            path.push_back(verticesSucRes.at(static_cast<const unsigned long>(res)));
    }
    return path;
}

LaneletGraph::LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    for (const auto &let : lanelets) {
        verticesAdjSuc.insert({let->getId(), graphAdjSuc.add_vertex(let->getId())});
        verticesSuc.insert({let->getId(), graphSuc.add_vertex(let->getId())});
        verticesAdjSucRes.insert({verticesAdjSuc.at(let->getId()), let->getId()});
        verticesSucRes.insert({verticesSuc.at(let->getId()), let->getId()});
    }

    for (const auto &let : lanelets) {
        if (let->getAdjacentLeft().adj != nullptr and let->getAdjacentLeft().dir == DrivingDirection::same) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentLeft().adj->getId())), 4);
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentLeft().adj->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())), 4);
        }
        if (let->getAdjacentRight().adj != nullptr and let->getAdjacentRight().dir == DrivingDirection::same) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentRight().adj->getId())), 4);
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentRight().adj->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())), 4);
        }
        for (const auto &suc : let->getSuccessors()) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(suc->getId())), 1);
            graphSuc.add_edge(static_cast<ptrdiff_t>(verticesSuc.at(let->getId())),
                              static_cast<ptrdiff_t>(verticesSuc.at(suc->getId())), 1);
        }
    }
}
