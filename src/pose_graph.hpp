#ifndef POSE_GRAPH_HPP_
#define POSE_GRAPH_HPP_

#include "pose.hpp"
#include "utils.hpp"
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <spdlog/spdlog.h>
#include <unordered_set>
#include <vector>

namespace slammy::pose_graph {

struct Keyframe;
struct PoseGraph;
struct Point;

struct Point {
  std::vector<Keyframe *> observers;
  Eigen::Vector3d t;

  Point(Eigen::Vector3d t) : t(t) {}

  Point &seen_from(std::unique_ptr<Keyframe> const &keyframe);
};

struct Keyframe {
  cv::Mat frame;
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  slammy::pose::Pose<slammy::pose::Camera, slammy::pose::World> pose;
  std::unordered_set<Point *> points;

  Keyframe(
      cv::Mat const frame, cv::Mat const descriptors,
      std::vector<cv::KeyPoint> const keypoints,
      slammy::pose::Pose<slammy::pose::Camera, slammy::pose::World> const &pose)
      : frame(frame), descriptors(descriptors), keypoints(keypoints),
        pose(pose) {}
};

struct PointAdder {
  template <typename... Args>
  PointAdder(PoseGraph &pg, Args &&...args)
      : point(std::make_unique<Point>(std::forward<Args>(args)...)), pg(pg) {}

  ~PointAdder() { ASS_EQ(point, nullptr); }

  void build();

  std::unique_ptr<Point> point;
  PoseGraph &pg;
};

struct PoseGraph {
  std::unordered_set<std::unique_ptr<Point>> points;
  std::vector<std::unique_ptr<Keyframe>> keyframes;

  template <typename... Args> Point &add_point(Args &&...args) {
    return **points
                 .emplace(std::make_unique<Point>(std::forward<Args>(args)...))
                 .first;
  }
};

inline Point &Point::seen_from(std::unique_ptr<Keyframe> const &keyframe) {
  observers.emplace_back(keyframe.get());
  keyframe->points.emplace(this);
  return *this;
}
} // namespace slammy::pose_graph

#endif // POSE_GRAPH_HPP_
