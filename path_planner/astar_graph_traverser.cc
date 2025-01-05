#include "astar_graph_traverser.h"

#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>

namespace path_planner {

ASTARGraphTraverser::ASTARGraphTraverser()
    : costed_neighbors_(GenerateCostedNeighbors()) {}

void ASTARGraphTraverser::SetMap(const Map& map) { map_ = map; }

std::optional<Path> ASTARGraphTraverser::GeneratePath(
    const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
  if (!IsWithinMap(start, map_.dimension) || !IsWithinMap(end, map_.dimension))
    return std::nullopt;
  if (start == end) return std::nullopt;
  Path path = GeneratePathByASTAR(start, end);
  if (path.path.size() == 0) return std::nullopt;
  return path;
}

std::deque<Eigen::Vector2i> ASTARGraphTraverser::GetVisitQueue() {
  return visit_queue_;
}

Path ASTARGraphTraverser::GeneratePathByASTAR(const Eigen::Vector2i& start,
                                              const Eigen::Vector2i& end) {
  bool path_found = false;
  const auto width = map_.dimension.x();
  const auto height = map_.dimension.y();
  std::vector<Cell> cost_map(width * height);
  std::priority_queue<Cell, std::vector<Cell>, Compare> prioritized_open_list;
  std::set<int> closed_list;

  auto flat_index = ToFlatIndex(start, width);
  auto& start_cell = cost_map[flat_index];
  start_cell.position = start;
  start_cell.parent = nullptr;
  start_cell.cumulative_cost = 0;
  start_cell.heuristic_cost = 10 * (end - start).lpNorm<1>();
  prioritized_open_list.push(start_cell);

  while (!prioritized_open_list.empty()) {
    const auto current_cell = prioritized_open_list.top();
    prioritized_open_list.pop();
    auto current_index = ToFlatIndex(current_cell.position, width);

    if (closed_list.count(current_index)) continue;
    visit_queue_.push_back(current_cell.position);
    closed_list.emplace(current_index);

    const auto& current_position = current_cell.position;

    for (const auto& costed_neighbor : costed_neighbors_) {
      auto neighbor_position = current_position + costed_neighbor.position;
      auto neighbor_flat_index = ToFlatIndex(neighbor_position, width);
      if (closed_list.count(neighbor_flat_index) ||
          !IsWithinMap(neighbor_position, map_.dimension) ||
          !map_.grid[neighbor_flat_index])
        continue;

      auto& neighbor_cell = cost_map[neighbor_flat_index];
      neighbor_cell.position = neighbor_position;
      neighbor_cell.heuristic_cost = 10 * (end - neighbor_position).lpNorm<1>();
      if (neighbor_cell.cumulative_cost >
          current_cell.cumulative_cost + costed_neighbor.cost) {
        neighbor_cell.cumulative_cost =
            current_cell.cumulative_cost + costed_neighbor.cost;
        neighbor_cell.parent = &cost_map[current_index];
        prioritized_open_list.push(neighbor_cell);
      }

      if (neighbor_position == end) {
        path_found = true;
        break;
      }
    }
    if (path_found) break;
  }

  Path path;
  path.start = start;
  path.destination = end;
  if (path_found) {
    const auto end_flat_index = ToFlatIndex(end, width);
    auto* next = &cost_map[end_flat_index];
    while (next != nullptr) {
      path.path.push_back(next->position);
      next = next->parent;
    }
    std::cerr << "Path length: " << path.path.size() << std::endl;
    return path;
  }
  std::cerr << "Path not found by ASTAR";
  return path;
}

};  // namespace path_planner