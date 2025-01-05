#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <deque>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "graph_traverser_interface.h"

namespace path_planner {

enum class Mode { BFS, ASTAR, DSTAR };

struct Parameters {
  Mode mode{Mode::BFS};
  float max_clearance{10.f};
};

class PathPlanner {
 public:
  PathPlanner(const Parameters& parameters);
  void SetMap(const Map& map);
  std::optional<Path> GeneratePath(const Eigen::Vector2i& start,
                                   const Eigen::Vector2i& end);
  std::deque<Eigen::Vector2i> GetVisitQueue();
  std::vector<std::pair<Eigen::Vector2i, float>> GetClearanceBandCells();

 private:
  struct PositionHash {
    uint64_t operator()(const Eigen::Vector2i& position) const {
      return static_cast<uint64_t>(position.x()) << 32 |
             static_cast<uint64_t>(position.y());
    }
  };

  struct PositionCompare {
    bool operator()(const Eigen::Vector2i& lhs,
                    const Eigen::Vector2i& rhs) const {
      return (lhs.x() < rhs.x()) ||
             ((!(rhs.x() < lhs.x())) && (lhs.y() < rhs.y()));
    }
  };
  std::unordered_map<Eigen::Vector2i, float, PositionHash>
  GenerateClearanceBand(const Path& path, const float max_clearance);
  std::unordered_map<Eigen::Vector2i, float, PositionHash>
  NaivelyGenerateClearanceBand(const Path& path, const float max_clearance);
  std::vector<Eigen::Vector2i> GenerateIndiceInCircle(const float radius);
  const Parameters parameters_;
  std::unique_ptr<Map> map_;
  std::unique_ptr<GraphTraverserInterface> graph_traverser_;
  std::unordered_map<Eigen::Vector2i, float, PositionHash>
      clearance_band_cells_;
};

}  // namespace path_planner
#endif