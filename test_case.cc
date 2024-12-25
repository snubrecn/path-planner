#include "test_case.h"

namespace test_case {
TestCase::TestCase(const int width, const int height,
                   const Eigen::Vector2i& start, const Eigen::Vector2i& end)
    : start_(start), end_(end) {
    map_.dimension = Eigen::Vector2i(width, height);
    map_.grid.assign(width * height, true);
    visualization_map_ = cv::Mat(height, width, CV_8UC3);
    for (auto col = 0; col < width; ++col) {
        for (auto row = 0; row < height; ++row) {
            visualization_map_.at<cv::Vec3b>(row, col) =
                cv::Vec3b(255, 255, 255);
        }
    }
}

void TestCase::SetObstaclePoints(
    const std::vector<Eigen::Vector2i>& obstacle_points) {
    const auto width = map_.dimension.x();
    for (const auto& obstacle_point : obstacle_points) {
        map_.grid[obstacle_point.x() + width * obstacle_point.y()] = false;
        visualization_map_.at<cv::Vec3b>(
            obstacle_point.y(), obstacle_point.x()) = cv::Vec3b(0, 0, 0);
    }
}

void TestCase::SetPath(const std::deque<Eigen::Vector2i>& path) {
    for (const auto& waypoint : path)
        visualization_map_.at<cv::Vec3b>(waypoint.y(), waypoint.x()) =
            cv::Vec3b(0, 255, 0);
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
        }
    }

    cv::Mat image = visualization_map_;
    if (resize_factor != 1.0) {
        cv::resize(visualization_map_, image,
                   cv::Size(visualization_map_.rows * resize_factor,
                            visualization_map_.cols * resize_factor));
    }

    cv::imshow("path_planning_result", image);
    cv::waitKey(0);
}

};  // namespace test_case