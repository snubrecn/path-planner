#include <chrono>
#include <deque>
#include <iostream>

#include "Eigen/Geometry"
#include "path_planner/path_planner.h"
#include "test_case.h"

int main(void) {
  path_planner::Parameters parameters;
  parameters.mode = path_planner::Mode::ASTAR;
  path_planner::PathPlanner path_planner(parameters);
  std::chrono::steady_clock::time_point ts, te;

  test_case::TestCase tc_3(1200, 1200, Eigen::Vector2i(130, 140),
                           Eigen::Vector2i(1090, 953));
  tc_3.SetRandomObstaclePoints(300, 20);
  path_planner.SetMap(tc_3.GetMap());
  ts = std::chrono::steady_clock::now();
  auto path = path_planner.GeneratePath(tc_3.GetStart(), tc_3.GetEnd());
  te = std::chrono::steady_clock::now();
  auto duration = (te - ts).count() * 1e-6;
  std::cerr << "tc 3 path generation took " << duration << " ms\n";
  if (path.has_value()) {
    tc_3.SetPath(path->path);
    tc_3.SetVisitQueue(path_planner.GetVisitQueue());
  }
  tc_3.VisualizeMap(0.5);

  return 0;
}
