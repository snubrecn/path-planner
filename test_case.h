#ifndef TEST_CASE_H_
#define TEST_CASE_H_

#include <deque>
#include <vector>

#include "Eigen/Geometry"
#include "opencv4/opencv2/opencv.hpp"
#include "path_planner/path_planner.h"

namespace test_case {

class TestCase {
   public:
    TestCase(const int width, const int height, const Eigen::Vector2i& start,
             const Eigen::Vector2i& end);
    void SetObstaclePoints(const std::vector<Eigen::Vector2i>& obstacle_points);
    void SetPath(const std::deque<Eigen::Vector2i>& path);

    path_planner::Map GetMap();
    Eigen::Vector2i GetStart();
    Eigen::Vector2i GetEnd();
    void VisualizeMap(const double resize_factor = 1.0);

   private:
    Eigen::Vector2i start_;
    Eigen::Vector2i end_;
    path_planner::Map map_;
    cv::Mat visualization_map_;
};

};  // namespace test_case
#endif