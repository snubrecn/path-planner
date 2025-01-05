#include "path_planner.h"

#include <chrono>
#include <iostream>
#include <set>

#include "astar_graph_traverser.h"
#include "bfs_graph_traverser.h"
#include "dstar_graph_traverser.h"

namespace path_planner {
PathPlanner::PathPlanner(const Parameters& parameters)
    : parameters_(parameters) {}

void PathPlanner::SetMap(const Map& map) {
  map_ = std::make_unique<Map>();
  map_->dimension = map.dimension;
  map_->grid = map.grid;
}

std::optional<Path> PathPlanner::GeneratePath(const Eigen::Vector2i& start,
                                              const Eigen::Vector2i& end) {
  if (map_ == nullptr) return std::nullopt;
  const auto map_width = map_->dimension.x();
  if (!map_->grid[ToFlatIndex(start, map_width)] ||
      !map_->grid[ToFlatIndex(end, map_width)]) {
    std::cerr << "Either start or end is/are located on obstacle\n";
    return std::nullopt;
  }

  std::optional<Path> path;
  switch (parameters_.mode) {
    case Mode::BFS:
      graph_traverser_ = std::make_unique<BFSGraphTraverser>();
      break;
    case Mode::ASTAR:
      graph_traverser_ = std::make_unique<ASTARGraphTraverser>();
      break;
    case Mode::DSTAR:
      graph_traverser_ = std::make_unique<DSTARGraphTraverser>();
      break;
    default:
      break;
  }

  graph_traverser_->SetMap(*map_);
  auto ts = std::chrono::steady_clock::now();
  path = graph_traverser_->GeneratePath(start, end);
  auto te = std::chrono::steady_clock::now();
  auto duration = (te - ts).count() * 1e-6;
  std::cerr << "Path generation took " << duration << " ms\n";

  if (path.has_value()) {
    auto ts = std::chrono::steady_clock::now();
    clearance_band_cells_ =
        NaivelyGenerateClearanceBand(path.value(), parameters_.max_clearance);
    auto te = std::chrono::steady_clock::now();
    auto duration = (te - ts).count() * 1e-6;
    std::cerr << "Clearance band generation took " << duration << " ms\n";
  }
  return path;
}

std::deque<Eigen::Vector2i> PathPlanner::GetVisitQueue() {
  if (graph_traverser_) return graph_traverser_->GetVisitQueue();
  return {};
}

std::vector<std::pair<Eigen::Vector2i, float>>
PathPlanner::GetClearanceBandCells() {
  if (clearance_band_cells_.size() == 0) return {};
  std::vector<std::pair<Eigen::Vector2i, float>> clearance_band_cells;
  clearance_band_cells.reserve(clearance_band_cells_.size());
  const auto width = map_->dimension.x();
  for (const auto& [position, distance] : clearance_band_cells_)
    clearance_band_cells.push_back(std::make_pair(position, distance));

  return clearance_band_cells;
}

std::unordered_map<Eigen::Vector2i, float, PathPlanner::PositionHash>
PathPlanner::GenerateClearanceBand(const Path& path,
                                   const float max_clearance) {
  std::unordered_map<Eigen::Vector2i, float, PositionHash> clearance_band_cells;

  const auto& neighbor_positions = GenerateNeighborPositions();
  const auto& indice_in_circle = GenerateIndiceInCircle(max_clearance);
  const auto& grid = map_->grid;
  const auto& map_dimension = map_->dimension;
  const auto& map_width = map_->dimension.x();

  for (const auto& waypoint : path.path) {
    for (const auto& index_in_circle : indice_in_circle) {
      const Eigen::Vector2i current_position = waypoint + index_in_circle;
      if (!IsWithinMap(current_position, map_dimension)) continue;
      if (!grid[ToFlatIndex(current_position, map_width)]) {
        std::deque<Eigen::Vector2i> bfs;
        bfs.push_back(current_position);
        while (!bfs.empty()) {
          const Eigen::Vector2i bfs_current_position = bfs.front();
          bfs.pop_front();
          for (const auto& neighbor_position : neighbor_positions) {
            const Eigen::Vector2i bfs_neighbor_position =
                bfs_current_position + neighbor_position;
            if (!IsWithinMap(bfs_current_position, map_dimension)) continue;
            if (!grid[ToFlatIndex(bfs_neighbor_position, map_width)]) continue;
            if (clearance_band_cells.count(bfs_neighbor_position) == 0)
              clearance_band_cells[bfs_neighbor_position] = max_clearance;
            const float distance =
                (bfs_neighbor_position - current_position).cast<float>().norm();
            if (clearance_band_cells[bfs_neighbor_position] > distance) {
              clearance_band_cells[bfs_neighbor_position] = distance;
              bfs.push_back(bfs_neighbor_position);
            }
          }
        }
      } else {
        if (clearance_band_cells.count(current_position)) continue;
        clearance_band_cells[current_position] = max_clearance;
      }
    }
  }
  return clearance_band_cells;
}

std::unordered_map<Eigen::Vector2i, float, PathPlanner::PositionHash>
PathPlanner::NaivelyGenerateClearanceBand(const Path& path,
                                          const float max_clearance) {
  std::unordered_map<Eigen::Vector2i, float, PositionHash> clearance_band_cells;

  const auto& neighbor_positions = GenerateNeighborPositions();
  const auto& indice_in_circle = GenerateIndiceInCircle(max_clearance);
  const auto& grid = map_->grid;
  const auto& map_dimension = map_->dimension;
  const auto& map_width = map_->dimension.x();

  for (const auto& waypoint : path.path) {
    for (const auto& index_in_circle : indice_in_circle) {
      const Eigen::Vector2i current_position = waypoint + index_in_circle;
      if (!IsWithinMap(current_position, map_dimension)) continue;
      if (!grid[ToFlatIndex(current_position, map_width)]) {
        for (const auto& nested_index_in_circle : indice_in_circle) {
          const Eigen::Vector2i obstacle_neighbor_position =
              current_position + nested_index_in_circle;
          if (!IsWithinMap(obstacle_neighbor_position, map_dimension)) continue;
          if (!grid[ToFlatIndex(obstacle_neighbor_position, map_width)])
            continue;
          // if ((obstacle_neighbor_position - waypoint).cast<float>().norm() >
          //     max_clearance)
          //   continue;
          if (clearance_band_cells.count(obstacle_neighbor_position) == 0)
            clearance_band_cells[obstacle_neighbor_position] = max_clearance;
          const float distance = nested_index_in_circle.cast<float>().norm();
          if (clearance_band_cells[obstacle_neighbor_position] > distance)
            clearance_band_cells[obstacle_neighbor_position] = distance;
        }
      } else {
        if (clearance_band_cells.count(current_position)) continue;
        clearance_band_cells[current_position] = max_clearance;
      }
    }
  }
  return clearance_band_cells;
}

std::vector<Eigen::Vector2i> PathPlanner::GenerateIndiceInCircle(
    const float radius) {
  const int radius_i = static_cast<int>(radius);
  std::vector<Eigen::Vector2i> indice_in_circle;
  for (auto dx = -radius_i; dx <= radius_i; ++dx) {
    for (auto dy = -radius_i; dy <= radius_i; ++dy) {
      if (dx * dx + dy * dy > radius * radius) continue;
      indice_in_circle.push_back(Eigen::Vector2i(dx, dy));
    }
  }
  return indice_in_circle;
}

};  // namespace path_planner