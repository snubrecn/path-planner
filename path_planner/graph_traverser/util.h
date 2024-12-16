#ifndef PATH_PLANNER_GRAPH_TRAVERSER_UTIL_H_
#define PATH_PLANNER_GRAPH_TRAVERSER_UTIL_H_

#include <vector>

#include "Eigen/Geometry"

namespace path_planner {
namespace graph_traverser {
namespace {

int ToFlatIndex(const Eigen::Vector2i& pos, const int width) {
    return pos.x() + pos.y() * width;
}

std::vector<Eigen::Vector2i> GenerateNeighborPositions(
    const Eigen::Vector2i& center) {
    std::vector<Eigen::Vector2i> neighbor_positions;
    for (auto dx = -1; dx <= 1; ++dx) {
        for (auto dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            neighbor_positions.push_back(center + Eigen::Vector2i(dx, dy));
        }
    }
    return neighbor_positions;
}

bool IsWithinMap(const Eigen::Vector2i& pos, const Eigen::Vector2i& dimension) {
    return pos.x() >= 0 && pos.x() < dimension.x() && pos.y() >= 0 &&
           pos.y() < dimension.y();
}

}  // namespace
}  // namespace graph_traverser
}  // namespace path_planner

#endif