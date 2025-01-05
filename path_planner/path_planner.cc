#include "path_planner.h"

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
  return graph_traverser_->GeneratePath(start, end);
}

std::deque<Eigen::Vector2i> PathPlanner::GetVisitQueue() {
  if (graph_traverser_) return graph_traverser_->GetVisitQueue();
  return {};
}

};  // namespace path_planner