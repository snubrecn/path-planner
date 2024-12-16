#ifndef PATH_PLANNER_GRAPH_TRAVERSER_INTERFACE_H_
#define PATH_PLANNER_GRAPH_TRAVERSER_INTERFACE_H_

#include <deque>
#include <optional>
#include <vector>

#include "Eigen/Geometry"

namespace path_planner {
namespace graph_traverser {

struct Map {
    Eigen::Vector2i dimension;
    std::vector<bool> grid;  // true: freespace, false: occupied
};

struct Path {
    Eigen::Vector2i start;
    Eigen::Vector2i destination;
    std::deque<Eigen::Vector2i> path;
};

class GraphTraverserInterface {
   public:
    GraphTraverserInterface() {}
    virtual void SetMap(const Map& map) = 0;
    virtual std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                             const Eigen::Vector2i& end) = 0;
};
}  // namespace graph_traverser
}  // namespace path_planner

#endif