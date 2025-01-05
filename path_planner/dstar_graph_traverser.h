#ifndef PATH_PLANNER_DSTAR_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_DSTAR_GRAPH_TRAVERSER_H_

#include "graph_traverser_interface.h"

namespace path_planner {

class DSTARGraphTraverser : public GraphTraverserInterface {
 public:
  DSTARGraphTraverser();
  void SetMap(const Map& map) override;
  std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                   const Eigen::Vector2i& end) override;
  std::deque<Eigen::Vector2i> GetVisitQueue() override;
};

}  // namespace path_planner

#endif