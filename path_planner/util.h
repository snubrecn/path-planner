#ifndef PATH_PLANNER_GRAPH_TRAVERSER_UTIL_H_
#define PATH_PLANNER_GRAPH_TRAVERSER_UTIL_H_

#include <vector>

#include "Eigen/Geometry"

namespace path_planner {

struct CostedNeighbor {
  Eigen::Vector2i position;
  int cost{0};
};

namespace {

inline int ToFlatIndex(const Eigen::Vector2i& grid_index, const int width) {
  return grid_index.x() + grid_index.y() * width;
}

inline Eigen::Vector2i ToGridIndex(const int flat_index, const int width) {
  return Eigen::Vector2i(flat_index % width, flat_index / width);
}

std::vector<Eigen::Vector2i> GenerateNeighborPositions() {
  std::vector<Eigen::Vector2i> neighbor_positions;
  for (auto dx = -1; dx <= 1; ++dx) {
    for (auto dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      neighbor_positions.push_back(Eigen::Vector2i(dx, dy));
    }
  }
  return neighbor_positions;
}

std::vector<CostedNeighbor> GenerateCostedNeighbors() {
  std::vector<CostedNeighbor> costed_neighbors;
  for (auto dx = -1; dx <= 1; ++dx) {
    for (auto dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      CostedNeighbor costed_neighbor;
      costed_neighbor.position = Eigen::Vector2i(dx, dy);
      costed_neighbor.cost = (dx == 0 || dy == 0) ? 10 : 14;
      costed_neighbors.push_back(costed_neighbor);
    }
  }
  return costed_neighbors;
};

inline bool IsWithinMap(const Eigen::Vector2i& grid_index,
                        const Eigen::Vector2i& dimension) {
  return grid_index.x() >= 0 && grid_index.x() < dimension.x() &&
         grid_index.y() >= 0 && grid_index.y() < dimension.y();
}

}  // namespace
}  // namespace path_planner

#endif