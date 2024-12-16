#include "astar_graph_traverser.h"

#include "util.h"

namespace path_planner {
namespace graph_traverser {
namespace astar {

ASTARGraphTraverser::ASTARGraphTraverser() {}

void ASTARGraphTraverser::SetMap(const Map& map) {}

std::optional<Path> ASTARGraphTraverser::GeneratePath(
    const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
    return std::nullopt;
}

}  // namespace astar
}  // namespace graph_traverser
};  // namespace path_planner