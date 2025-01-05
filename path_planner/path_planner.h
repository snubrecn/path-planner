#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "Eigen/Geometry"
#include "graph_traverser_interface.h"

namespace path_planner {

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
  std::deque<Eigen::Vector2i> GetVisitQueue();

 private:
  const Parameters parameters_;
  std::unique_ptr<Map> map_;
  std::unique_ptr<GraphTraverserInterface> graph_traverser_;
};

}  // namespace path_planner
#endif