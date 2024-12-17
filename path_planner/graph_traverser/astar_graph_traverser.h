#ifndef PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_

#include <cinttypes>
#include <vector>

#include "graph_traverser_interface.h"

namespace path_planner {
namespace graph_traverser {
namespace astar {

struct Cell {
    Eigen::Vector2i position;
    int heuristic_cost{std::numeric_limits<int>::max()};
    int cumulative_cost{std::numeric_limits<int>::max()};
    Cell* parent{nullptr};
};

struct Neighbor {
    Eigen::Vector2i move;
    int move_cost{0};
};

struct Compare {
    bool operator()(const Cell& lhs, const Cell& rhs) {
        return lhs.heuristic_cost + lhs.cumulative_cost >
               rhs.heuristic_cost + rhs.cumulative_cost;
    }
};

class ASTARGraphTraverser : public GraphTraverserInterface {
   public:
    ASTARGraphTraverser();
    void SetMap(const Map& map) override;
    std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                     const Eigen::Vector2i& end) override;

   private:
    std::vector<Neighbor> GenerateNeighbors();
    Path GeneratePathByASTAR(const Eigen::Vector2i& start,
                             const Eigen::Vector2i& end);
    std::vector<Neighbor> neighbors_;
    Map map_;
};

}  // namespace astar
}  // namespace graph_traverser
}  // namespace path_planner

#endif