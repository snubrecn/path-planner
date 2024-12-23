#include "bfs_graph_traverser.h"

#include <iostream>

#include "util.h"

namespace path_planner {
namespace graph_traverser {
namespace bfs {

BFSGraphTraverser::BFSGraphTraverser() {}

void BFSGraphTraverser::SetMap(const Map& map) { map_ = map; }

std::optional<Path> BFSGraphTraverser::GeneratePath(
    const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
    if (!IsWithinMap(start, map_.dimension) ||
        !IsWithinMap(end, map_.dimension))
        return std::nullopt;
    if (start == end) return std::nullopt;

    Path path = GeneratePathByBFS(start, end);
    if (path.path.size() == 0) return std::nullopt;
    return path;
}

Path BFSGraphTraverser::GeneratePathByBFS(const Eigen::Vector2i& start,
                                          const Eigen::Vector2i& end) {
    const auto width = map_.dimension.x();
    const auto height = map_.dimension.y();

    std::vector<Cell> cell_grid(width * height);
    std::deque<Eigen::Vector2i> queue;

    queue.push_back(start);
    const auto start_index = ToFlatIndex(start, width);
    cell_grid[start_index].position = start;
    cell_grid[start_index].visit = true;
    cell_grid[start_index].parent = nullptr;
    bool path_found = false;

    while (!queue.empty()) {
        const auto pos = queue.front();
        queue.pop_front();
        const auto flat_index = ToFlatIndex(pos, width);

        auto neighbor_positions = GenerateNeighborPositions(pos);
        for (const auto& neighbor_position : neighbor_positions) {
            if (!IsWithinMap(neighbor_position, map_.dimension)) continue;
            const auto neighbor_index = ToFlatIndex(neighbor_position, width);
            if (cell_grid[neighbor_index].visit || !map_.grid[neighbor_index])
                continue;
            cell_grid[neighbor_index].visit = true;
            cell_grid[neighbor_index].parent = &cell_grid[flat_index];
            cell_grid[neighbor_index].position = neighbor_position;
            if (neighbor_position == end) {
                path_found = true;
                break;
            }
            queue.push_back(neighbor_position);
        }
        if (path_found) break;
    }

    Path path;
    path.start = start;
    path.destination = end;
    if (path_found) {
        const auto end_index = ToFlatIndex(end, width);
        auto* next = &cell_grid[end_index];
        while (next != nullptr) {
            path.path.push_back(next->position);
            next = next->parent;
        }
        std::cerr << "path length: " << path.path.size() << std::endl;
        return path;
    }
    std::cerr << "Path not found by BFS\n";
    return path;
}

}  // namespace bfs
}  // namespace graph_traverser
};  // namespace path_planner