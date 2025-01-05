#ifndef PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_

#include <cinttypes>
#include <deque>
#include <vector>

#include "graph_traverser_interface.h"
#include "util.h"

namespace path_planner {

class ASTARGraphTraverser : public GraphTraverserInterface {
 public:
  ASTARGraphTraverser();
  void SetMap(const Map& map) override;
  std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                   const Eigen::Vector2i& end) override;
  std::deque<Eigen::Vector2i> GetVisitQueue() override;

 private:
  struct Cell {
    Eigen::Vector2i position;
    int heuristic_cost{std::numeric_limits<int>::max()};
    int cumulative_cost{std::numeric_limits<int>::max()};
    Cell* parent{nullptr};
  };

  struct Compare {
    bool operator()(const Cell& lhs, const Cell& rhs) {
      return lhs.heuristic_cost + lhs.cumulative_cost >
             rhs.heuristic_cost + rhs.cumulative_cost;
    }
  };

  Path GeneratePathByASTAR(const Eigen::Vector2i& start,
                           const Eigen::Vector2i& end);
  std::vector<CostedNeighbor> costed_neighbors_;
  Map map_;
  std::deque<Eigen::Vector2i> visit_queue_;
};

}  // namespace path_planner

#endif