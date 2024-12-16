#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "Eigen/Geometry"
#include "graph_traverser/graph_traverser_interface.h"

namespace path_planner {

struct Map {
    Eigen::Vector2i dimension;
    std::vector<bool> grid;  // true: freespace, false: occupied
};

struct Path {
    Eigen::Vector2i start;
    Eigen::Vector2i destination;
    std::deque<Eigen::Vector2i> path;
};

enum class Mode { BFS, ASTAR, DSTAR };

struct Parameters {
    Mode mode{Mode::BFS};
};

class PathPlanner {
   public:
    PathPlanner(const Parameters& parameters);
    void SetMap(const Map& map);
    std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                     const Eigen::Vector2i& end);

   private:
    const Parameters parameters_;
    std::unique_ptr<graph_traverser::Map> map_;
    std::unique_ptr<graph_traverser::GraphTraverserInterface> graph_traverser_;
};

}  // namespace path_planner
#endif