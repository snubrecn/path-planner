#include <deque>
#include <iostream>

#include "Eigen/Geometry"
#include "path_planner/path_planner.h"
#include "test_case.h"

int main(void) {
    test_case::TestCase tc_1(600, 600, Eigen::Vector2i(250, 200),
                             Eigen::Vector2i(400, 400));

    std::vector<Eigen::Vector2i> obstacle_points;
    for (auto x = 50; x < 550; ++x) {
        obstacle_points.push_back(Eigen::Vector2i(x, 300));
        obstacle_points.push_back(Eigen::Vector2i(300, x));
    }
    tc_1.SetObstaclePoints(obstacle_points);

    path_planner::Parameters parameters;
    path_planner::PathPlanner path_planner(parameters);
    path_planner.SetMap(tc_1.GetMap());
    auto path = path_planner.GeneratePath(tc_1.GetStart(), tc_1.GetEnd());
    if (path.has_value()) tc_1.SetPath(path->path);
    tc_1.VisualizeMap();

    return 0;
}
