#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include "magic_enum.hpp"
#include "pose.hpp"
#include "pose_graph.hpp"
#include "utils.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/util/ForwardDeclarations.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <spdlog/spdlog.h>

namespace slammy::tracker {

// TODO file-ify
auto constexpr MIN_KPTS = 500;
auto constexpr MIN_MATCHES = 100;

enum class TrackerState { Initializing, Tracking, Lost };

class Tracker {
public:
  Tracker(double fx, double fy, double cx, double cy);

  void next(cv::Mat const &next_frame);

private:
  void initialise(cv::Mat const &next_frame);

  void triangulate_points();

  cv::Ptr<cv::ORB> _orb;
  cv::Ptr<cv::BFMatcher> _matcher;
  cv::Mat _k_cv;
  Eigen::Matrix3d _k;
  Eigen::Matrix3d _k_inv;

  std::vector<cv::Point2f> _pts1;
  std::vector<cv::Point2f> _pts2;
  std::vector<cv::KeyPoint> _kpts_buf;
  cv::Mat _desc_buf;
  std::vector<cv::DMatch> _matches_buf;

  unsigned int _counter = 0;
  unsigned int _fail_counter = 0;

  TrackerState _state = TrackerState::Initializing;
  slammy::pose_graph::PoseGraph _pg;
};

} // namespace slammy::tracker
#endif // TRACKER_HPP_
