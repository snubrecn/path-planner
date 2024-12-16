#include <deque>
#include <iostream>

#include "Eigen/Geometry"
#include "opencv4/opencv2/opencv.hpp"
#include "path_planner/path_planner.h"

int main(void) {
    const auto width = 600;
    const auto height = 600;
    std::vector<bool> example_map(width * height, true);

    Eigen::Vector2i start(250, 200);
    Eigen::Vector2i end(400, 450);

    for (auto x = 50; x < 550; ++x) {
        auto flat_index = x + 300 * width;
        example_map[flat_index] = false;
    }
    for (auto y = 50; y < 550; ++y) {
        auto flat_index = 300 + y * width;
        example_map[flat_index] = false;
    }

    path_planner::Parameters parameters;
    path_planner::PathPlanner path_planner(parameters);
    path_planner::Map map;
    map.dimension = Eigen::Vector2i(width, height);
    map.grid = example_map;
    path_planner.SetMap(map);
    auto path = path_planner.GeneratePath(start, end);
    std::deque<Eigen::Vector2i> path_list;
    if (path.has_value()) path_list = path->path;

    cv::Mat map_image(height, width, CV_8UC3);
    for (auto cols = 0; cols < width; ++cols) {
        for (auto rows = 0; rows < height; ++rows) {
            map_image.at<cv::Vec3b>(rows, cols) = cv::Vec3b(255, 255, 255);
        }
    }

    for (auto x = 50; x < 550; ++x) {
        map_image.at<cv::Vec3b>(300, x) = cv::Vec3b(0, 0, 0);
        map_image.at<cv::Vec3b>(x, 300) = cv::Vec3b(0, 0, 0);
    }

    map_image.at<cv::Vec3b>(start.y(), start.x()) = cv::Vec3b(255, 0, 0);
    map_image.at<cv::Vec3b>(end.y(), end.x()) = cv::Vec3b(0, 0, 255);

    while (!path_list.empty()) {
        const auto point = path_list.front();
        path_list.pop_front();
        map_image.at<cv::Vec3b>(point.y(), point.x()) = cv::Vec3b(0, 255, 0);
    }

    cv::imshow("path_planning", map_image);
    cv::waitKey(0);

    return 0;
}
