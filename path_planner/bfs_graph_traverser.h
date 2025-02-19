#ifndef PATH_PLANNER_BFS_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_BFS_GRAPH_TRAVERSER_H_

#include "graph_traverser_interface.h"

namespace path_planner {

class BFSGraphTraverser : public GraphTraverserInterface {
 public:
  BFSGraphTraverser();
  void SetMap(const Map& map) override;
  std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                   const Eigen::Vector2i& end) override;
  std::deque<Eigen::Vector2i> GetVisitQueue() override;

 private:
  struct Cell {
    Eigen::Vector2i position;
    bool visit{false};
    Cell* parent{nullptr};
  };
  Path GeneratePathByBFS(const Eigen::Vector2i& start,
                         const Eigen::Vector2i& end);
  Map map_;
  std::deque<Eigen::Vector2i> visit_queue_;
};

}  // namespace path_planner

#endif