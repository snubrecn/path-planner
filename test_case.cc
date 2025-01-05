#include "test_case.h"

#include <random>

namespace {
std::random_device rd;
std::mt19937 gen(rd());
}  // namespace

namespace test_case {
TestCase::TestCase(const int width, const int height,
                   const Eigen::Vector2i& start, const Eigen::Vector2i& end)
    : start_(start), end_(end) {
  map_.dimension = Eigen::Vector2i(width, height);
  map_.grid.assign(width * height, true);
  visualization_map_ = cv::Mat(height, width, CV_8UC3);
  visit_queue_visualization_map_ = cv::Mat(height, width, CV_8UC3);
  clearance_band_visualization_map_ = cv::Mat(height, width, CV_8UC3);
  for (auto col = 0; col < width; ++col) {
    for (auto row = 0; row < height; ++row) {
      visualization_map_.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 255);
      visit_queue_visualization_map_.at<cv::Vec3b>(row, col) =
          cv::Vec3b(255, 255, 255);
      clearance_band_visualization_map_.at<cv::Vec3b>(row, col) =
          cv::Vec3b(255, 255, 255);
    }
  }
}

void TestCase::SetObstaclePoints(
    const std::vector<Eigen::Vector2i>& obstacle_points) {
  const auto width = map_.dimension.x();
  for (const auto& obstacle_point : obstacle_points) {
    map_.grid[obstacle_point.x() + width * obstacle_point.y()] = false;
    visualization_map_.at<cv::Vec3b>(obstacle_point.y(), obstacle_point.x()) =
        cv::Vec3b(0, 0, 0);
    visit_queue_visualization_map_.at<cv::Vec3b>(
        obstacle_point.y(), obstacle_point.x()) = cv::Vec3b(0, 0, 0);
    clearance_band_visualization_map_.at<cv::Vec3b>(
        obstacle_point.y(), obstacle_point.x()) = cv::Vec3b(0, 0, 255);
  }
}

void TestCase::SetRandomObstaclePoints(const int num_obstacles,
                                       const int inflation_thickness) {
  auto max_range = map_.dimension.x() * map_.dimension.y();
  std::uniform_int_distribution<int> dist(0, max_range);
  auto to_grid_index = [this](const int flat_index) {
    return Eigen::Vector2i(flat_index % map_.dimension.x(),
                           flat_index / map_.dimension.x());
  };
  auto to_flat_index = [this](const Eigen::Vector2i& grid_index) {
    return grid_index.x() + map_.dimension.x() * grid_index.y();
  };
  for (auto num = 0; num < num_obstacles; ++num) {
    auto random_index = dist(gen);
    Eigen::Vector2i obstacle_point = to_grid_index(random_index);
    if (obstacle_point == start_ || obstacle_point == end_) continue;
    for (auto dx = -inflation_thickness; dx <= inflation_thickness; ++dx) {
      for (auto dy = -inflation_thickness; dy <= inflation_thickness; ++dy) {
        Eigen::Vector2i neighbor_position =
            obstacle_point + Eigen::Vector2i(dx, dy);
        if (neighbor_position.x() < 0 ||
            neighbor_position.x() > map_.dimension.x() ||
            neighbor_position.y() < 0 ||
            neighbor_position.y() >= map_.dimension.y())
          continue;
        auto flat_index = to_flat_index(neighbor_position);
        map_.grid[flat_index] = false;
        visualization_map_.at<cv::Vec3b>(
            neighbor_position.y(), neighbor_position.x()) = cv::Vec3b(0, 0, 0);
        visit_queue_visualization_map_.at<cv::Vec3b>(
            neighbor_position.y(), neighbor_position.x()) = cv::Vec3b(0, 0, 0);
        clearance_band_visualization_map_.at<cv::Vec3b>(neighbor_position.y(),
                                                        neighbor_position.x()) =
            cv::Vec3b(0, 0, 255);
      }
    }
  }
}

void TestCase::SetPath(const std::deque<Eigen::Vector2i>& path) {
  for (const auto& waypoint : path)
    visualization_map_.at<cv::Vec3b>(waypoint.y(), waypoint.x()) =
        cv::Vec3b(0, 255, 0);
}

void TestCase::SetVisitQueue(const std::deque<Eigen::Vector2i>& visit_queue) {
  auto size = visit_queue.size();
  Eigen::Vector3d start_color(255, 0, 0);
  Eigen::Vector3d end_color(0, 0, 255);
  Eigen::Vector3d direction = end_color - start_color;
  auto norm = direction.norm();
  direction.normalize();
  Eigen::Vector3d step = direction * (norm / size);

  auto i = 0;
  for (const auto& visit : visit_queue) {
    Eigen::Vector3d color = start_color + i * step;
    visit_queue_visualization_map_.at<cv::Vec3b>(visit.y(), visit.x()) =
        cv::Vec3b(color.x(), color.y(), color.z());
    i++;
  }
}

void TestCase::SetClearanceBandCells(
    const std::vector<std::pair<Eigen::Vector2i, float>>& clearance_band_cells,
    const float max_clearance) {
  for (const auto& position_with_distance : clearance_band_cells) {
    const Eigen::Vector2i& position = position_with_distance.first;
    const auto distance = position_with_distance.second;
    const uint8_t intensity =
        static_cast<uint8_t>(255.0 * distance / max_clearance);
    clearance_band_visualization_map_.at<cv::Vec3b>(
        position.y(), position.x()) = cv::Vec3b(intensity, 0, 0);
  }
}

path_planner::Map TestCase::GetMap() { return map_; }

Eigen::Vector2i TestCase::GetStart() { return start_; }

Eigen::Vector2i TestCase::GetEnd() { return end_; }

void TestCase::VisualizeMap(const double resize_factor) {
  for (auto x = -1; x <= 1; ++x) {
    for (auto y = -1; y <= 1; ++y) {
      visualization_map_.at<cv::Vec3b>(start_.y() + y, start_.x() + x) =
          cv::Vec3b(255, 0, 0);
      visualization_map_.at<cv::Vec3b>(end_.y() + y, end_.x() + x) =
          cv::Vec3b(0, 0, 255);
      visit_queue_visualization_map_.at<cv::Vec3b>(
          start_.y() + y, start_.x() + x) = cv::Vec3b(255, 0, 0);
      visit_queue_visualization_map_.at<cv::Vec3b>(end_.y() + y, end_.x() + x) =
          cv::Vec3b(0, 0, 255);
    }
  }

  cv::Mat image;
  cv::hconcat(visualization_map_, visit_queue_visualization_map_, image);
  cv::Mat clearance_band_image = clearance_band_visualization_map_;
  if (resize_factor != 1.0) {
    cv::resize(
        image, image,
        cv::Size(image.cols * resize_factor, image.rows * resize_factor));
    cv::resize(clearance_band_image, clearance_band_image,
               cv::Size(clearance_band_image.cols * resize_factor,
                        clearance_band_image.rows * resize_factor));
  }

  cv::imshow("path_planning_result", image);
  cv::imshow("clearance band", clearance_band_image);
  cv::waitKey(0);
}

};  // namespace test_case