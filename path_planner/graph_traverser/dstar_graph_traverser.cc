#include "dstar_graph_traverser.h"

#include "util.h"

namespace path_planner {
namespace graph_traverser {
namespace dstar {

DSTARGraphTraverser::DSTARGraphTraverser() {}

void DSTARGraphTraverser::SetMap(const Map& map) {}

std::optional<Path> DSTARGraphTraverser::GeneratePath(
    const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
  return std::nullopt;
}

std::deque<Eigen::Vector2i> DSTARGraphTraverser::GetVisitQueue() { return {}; }

}  // namespace dstar
}  // namespace graph_traverser
};  // namespace path_planner