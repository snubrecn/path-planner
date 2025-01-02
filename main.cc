#include <chrono>
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
    parameters.mode = path_planner::Mode::ASTAR;
    path_planner::PathPlanner path_planner(parameters);
    path_planner.SetMap(tc_1.GetMap());
    auto ts = std::chrono::steady_clock::now();
    auto path = path_planner.GeneratePath(tc_1.GetStart(), tc_1.GetEnd());
    auto te = std::chrono::steady_clock::now();
    auto duration = (te - ts).count() * 1e-6;
    std::cerr << "tc 1 path generation took " << duration << " ms\n";
    if (path.has_value()) {
        tc_1.SetPath(path->path);
        tc_1.SetVisitQueue(path_planner.GetVisitQueue());
    }
    tc_1.VisualizeMap();

    test_case::TestCase tc_2(1000, 500, Eigen::Vector2i(400, 250),
                             Eigen::Vector2i(800, 250));
    obstacle_points.clear();
    for (auto x = 200; x < 300; ++x)
        obstacle_points.push_back(Eigen::Vector2i(500, x));
    tc_2.SetObstaclePoints(obstacle_points);

    path_planner.SetMap(tc_2.GetMap());
    ts = std::chrono::steady_clock::now();
    path = path_planner.GeneratePath(tc_2.GetStart(), tc_2.GetEnd());
    te = std::chrono::steady_clock::now();
    duration = (te - ts).count() * 1e-6;
    std::cerr << "tc 2 path generation took " << duration << " ms\n";
    if (path.has_value()) {
        tc_2.SetPath(path->path);
        tc_2.SetVisitQueue(path_planner.GetVisitQueue());
    }
    tc_2.VisualizeMap();

    test_case::TestCase tc_3(500, 500, Eigen::Vector2i(30, 40),
                             Eigen::Vector2i(430, 370));
    tc_3.SetRandomObstaclePoints(100, 3);
    path_planner.SetMap(tc_3.GetMap());
    ts = std::chrono::steady_clock::now();
    path = path_planner.GeneratePath(tc_3.GetStart(), tc_3.GetEnd());
    te = std::chrono::steady_clock::now();
    duration = (te - ts).count() * 1e-6;
    std::cerr << "tc 3 path generation took " << duration << " ms\n";
    if (path.has_value()) {
        tc_3.SetPath(path->path);
        tc_3.SetVisitQueue(path_planner.GetVisitQueue());
    }
    tc_3.VisualizeMap();

    return 0;
}
