#ifndef PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_ASTAR_GRAPH_TRAVERSER_H_

#include "graph_traverser_interface.h"

namespace path_planner {
namespace graph_traverser {
namespace astar {

class ASTARGraphTraverser : public GraphTraverserInterface {
   public:
    ASTARGraphTraverser();
    void SetMap(const Map& map) override;
    std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                     const Eigen::Vector2i& end) override;
};

}  // namespace astar
}  // namespace graph_traverser
}  // namespace path_planner

#endif