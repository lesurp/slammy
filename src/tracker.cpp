#include "tracker.hpp"
#include "vision.hpp"

using slammy::pose::Camera;
using slammy::pose::Pose;
using slammy::pose::World;
using slammy::pose_graph::Keyframe;

namespace slammy::tracker {
Tracker::Tracker(double fx, double fy, double cx, double cy)
    : _orb(cv::ORB::create()), _matcher(cv::BFMatcher::create()) {
  cv::Mat k = (cv::Mat_<double>(3, 3, CV_32F) << fx, 0.0, cx, 0.0, fy, cy, 0.0,
               0.0, 1.0);
  _k = Eigen::Matrix3d::Zero();
  _k(0, 0) = fx;
  _k(1, 1) = fy;
  _k(0, 2) = cx;
  _k(1, 2) = cy;
  _k(2, 2) = 1.0;

  _k_inv = Eigen::Matrix3d::Zero();
  _k_inv(0, 0) = 1 / fx;
  _k_inv(1, 1) = 1 / fy;
  _k_inv(0, 2) = -cx / fx;
  _k_inv(1, 2) = -cy / fy;
  _k_inv(2, 2) = 1.0;
}

void Tracker::next(cv::Mat const &next_frame) {
  spdlog::info("Current state: {}", magic_enum::enum_name(_state));
  switch (_state) {
  case TrackerState::Initializing:
    initialise(next_frame);
    break;
  }
}

void Tracker::initialise(cv::Mat const &next_frame) {
  ++_counter;
  spdlog::info("Counter: {}", _counter);

  _orb->detectAndCompute(next_frame, {}, _kpts_buf, _desc_buf);
  DBG_KPTS(next_frame, _kpts_buf);
  spdlog::info("Number of keypoints: {}", _kpts_buf.size());

  if (_kpts_buf.size() < MIN_KPTS) {
    spdlog::info("Not enough keypoints: skipping frame");
    return;
  }

  if (_pg.keyframes.empty()) {
    spdlog::info("First keyframe");
    _pg.keyframes.emplace_back(std::make_unique<Keyframe>(
        next_frame, _desc_buf.clone(), _kpts_buf, Pose<Camera, World>{}));
    return;
  }

  auto const &kf1 = *_pg.keyframes.front();
  auto const &descriptors1 = kf1.descriptors;
  auto const &keypoints1 = kf1.keypoints;

  _matcher->match(descriptors1, _desc_buf, _matches_buf);

  spdlog::info("Number of matches: {}", _matches_buf.size());
  if (_matches_buf.size() < MIN_MATCHES) {
    spdlog::info("Not enough matches, skipping (theshold: {})", MIN_MATCHES);
    ++_fail_counter;
    if (_fail_counter == 30) {
      spdlog::info(
          "Too many matched failed with the first keyframe - restting it");
      _pg.keyframes.clear();
      _fail_counter = 0;
      return;
    }
  }

  double cumsum_parallax = 0.0;
  for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
    auto const &pt1 = keypoints1[id1].pt;
    auto const &pt2 = _kpts_buf[id2].pt;
    _pts1.push_back(pt1);
    _pts2.push_back(pt2);
    auto delta = pt1 - pt2;
    cumsum_parallax += delta.x * delta.x + delta.y * delta.y;
  }

  double parallax = std::sqrt(cumsum_parallax);
  if (parallax < 10.0) {
    spdlog::info("Too little parallax between keyframes ({})", parallax);
    return;
  }

  auto p_2w = vision::relative_pose(_pts1, _pts2, _k_cv);

  _pg.keyframes.emplace_back(std::make_unique<Keyframe>(
      next_frame, _desc_buf.clone(), _kpts_buf, p_2w));
  triangulate_points();
}

void Tracker::triangulate_points() {
  ASS_EQ(_pg.keyframes.size(), 2);
  ASS_EQ(_pts1.size(), _pts2.size());

  Eigen::Matrix3d r_1w = _pg.keyframes[0]->pose.q.toRotationMatrix();
  Eigen::Matrix3d r_2w = _pg.keyframes[1]->pose.q.toRotationMatrix();
  Eigen::Vector3d w_1 = _pg.keyframes[0]->pose.t;
  Eigen::Vector3d w_2 = _pg.keyframes[1]->pose.t;

  Eigen::Matrix<double, 4, 3> a = Eigen::Matrix<double, 4, 3>::Zero();
  Eigen::Matrix<double, 4, 1> b = Eigen::Matrix<double, 4, 1>::Zero();
  for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
    auto const &pt1 = _pg.keyframes[0]->keypoints[id1].pt;
    auto const &pt2 = _pg.keyframes[1]->keypoints[id2].pt;

    Eigen::Vector2d mn_1 =
        (_k_inv * Eigen::Vector3d(pt1.x, pt2.y, 1)).head<2>();
    Eigen::Vector2d mn_2 =
        (_k_inv * Eigen::Vector3d(pt2.x, pt2.y, 1)).head<2>();

    a.row(0) = r_1w.row(0) - mn_1.x() * r_1w.row(2);
    a.row(1) = r_1w.row(1) - mn_1.y() * r_1w.row(2);
    a.row(2) = r_2w.row(2) - mn_2.x() * r_2w.row(2);
    a.row(3) = r_2w.row(3) - mn_2.y() * r_2w.row(2);

    b(0) = w_1.z() * mn_1.x() - w_1.x();
    b(1) = w_1.z() * mn_1.y() - w_1.y();
    b(2) = w_2.z() * mn_2.x() - w_2.x();
    b(3) = w_2.z() * mn_2.y() - w_2.y();

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 3>> svd(a);
    Eigen::Vector3d x = svd.solve(b);
    _pg.add_point(x)
        .seen_from(_pg.keyframes[0])
        .seen_from(_pg.keyframes[1]);
  }
}
} // namespace slammy::tracker
