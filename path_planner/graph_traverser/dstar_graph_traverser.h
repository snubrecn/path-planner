#ifndef PATH_PLANNER_DSTAR_GRAPH_TRAVERSER_H_
#define PATH_PLANNER_DSTAR_GRAPH_TRAVERSER_H_

#include "graph_traverser_interface.h"

namespace path_planner {
namespace graph_traverser {
namespace dstar {

class DSTARGraphTraverser : public GraphTraverserInterface {
   public:
    DSTARGraphTraverser();
    void SetMap(const Map& map) override;
    std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                     const Eigen::Vector2i& end) override;
    std::deque<Eigen::Vector2i> GetVisitQueue() override;
};

}  // namespace dstar
}  // namespace graph_traverser
}  // namespace path_planner

#endif