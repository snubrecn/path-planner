#ifndef TEST_CASE_H_
#define TEST_CASE_H_

#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "opencv4/opencv2/opencv.hpp"
#include "path_planner/path_planner.h"

namespace test_case {

class TestCase {
 public:
  TestCase(const int width, const int height, const Eigen::Vector2i& start,
           const Eigen::Vector2i& end);
  void SetMapTexture(const std::vector<bool>& map_texture);
  void SetObstaclePoints(const std::vector<Eigen::Vector2i>& obstacle_points);
  void SetRandomObstaclePoints(const int num_obstacles,
                               const int inflation_thickness = 1);
  void SetPath(const std::deque<Eigen::Vector2i>& path);
  void SetVisitQueue(const std::deque<Eigen::Vector2i>& visit_queue);
  void SetClearanceBandCells(
      const std::vector<std::pair<Eigen::Vector2i, float>>&
          clearance_band_cells,
      const float max_clearance);

  path_planner::Map GetMap();
  Eigen::Vector2i GetStart();
  Eigen::Vector2i GetEnd();
  void SaveMapTexture(const std::string& save_filepath);
  void LoadMapTexture(const std::string& load_filepath);
  void VisualizeMap(const double resize_factor = 1.0);

 private:
  Eigen::Vector2i start_;
  Eigen::Vector2i end_;
  path_planner::Map map_;
  cv::Mat visualization_map_;
  cv::Mat visit_queue_visualization_map_;
  cv::Mat clearance_band_visualization_map_;
};

};  // namespace test_case
#endif