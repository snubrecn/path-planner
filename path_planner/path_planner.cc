#include "path_planner.h"

#include "graph_traverser/astar_graph_traverser.h"
#include "graph_traverser/bfs_graph_traverser.h"
#include "graph_traverser/dstar_graph_traverser.h"

namespace path_planner {
PathPlanner::PathPlanner(const Parameters& parameters)
    : parameters_(parameters) {}

void PathPlanner::SetMap(const Map& map) {
    map_ = std::make_unique<graph_traverser::Map>();
    map_->dimension = map.dimension;
    map_->grid = map.grid;
}

std::optional<Path> PathPlanner::GeneratePath(const Eigen::Vector2i& start,
                                              const Eigen::Vector2i& end) {
    if (map_ == nullptr) return std::nullopt;
    std::optional<graph_traverser::Path> path;
    switch (parameters_.mode) {
        case Mode::BFS:
            graph_traverser_ =
                std::make_unique<graph_traverser::bfs::BFSGraphTraverser>();
            break;
        case Mode::ASTAR:
            graph_traverser_ =
                std::make_unique<graph_traverser::astar::ASTARGraphTraverser>();
            break;
        case Mode::DSTAR:
            graph_traverser_ =
                std::make_unique<graph_traverser::dstar::DSTARGraphTraverser>();
            break;
        default:
            break;
    }

    graph_traverser_->SetMap(*map_);
    path = graph_traverser_->GeneratePath(start, end);

    if (path.has_value()) {
        Path return_path;
        return_path.destination = path->destination;
        return_path.start = path->start;
        return_path.path = path->path;
        return return_path;
    }
    return std::nullopt;
}

};  // namespace path_planner