#include "tracker.hpp"
#include "pose.hpp"
#include "utils/debug.hpp"
#include "utils/fmt.hpp"
#include "utils/fn.hpp"
#include "vision.hpp"
#include <spdlog/spdlog.h>

using slammy::pose::PoseCW;
using slammy::pose_graph::Keyframe;
using slammy::utils::fn::map;

namespace slammy::tracker {
Tracker::Tracker(double fx, double fy, double cx, double cy)
    : _orb(cv::ORB::create()), _matcher(cv::BFMatcher::create()) {
  _k_cv = (cv::Mat_<double>(3, 3, CV_32F) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0,
           1.0);
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

SlammyState Tracker::next(cv::Mat const &next_frame) {
  spdlog::info("Current state: {}", magic_enum::enum_name(_state));
  switch (_state) {
  case TrackerState::Tracking:
    return track(next_frame);
  case TrackerState::Lost:
    return Lost{_p};
  case TrackerState::Initializing:
    if (initialise(next_frame)) {
      _fail_counter = 0;
      return JustInitialized{{_pg.keyframes[0]->pose, _pg.keyframes[1]->pose},
                             map<Eigen::Vector3d>(_pg.points)};
    } else {
      return Initializing{};
    }
  default:
    break;
  }
}

SlammyState Tracker::track(cv::Mat const &next_frame) {
  _orb->detectAndCompute(next_frame, {}, _kpts2, _desc2);
  DBG_KPTS(next_frame, _kpts2);
  spdlog::info("Number of keypoints: {}", _kpts2.size());

  if (_kpts2.size() < MIN_KPTS) {
    spdlog::info("Not enough keypoints: skipping frame");
    ++_fail_counter;
    if (_fail_counter > 10) {
      spdlog::warn("Tracking failed for {} - switching to lost_tracking!",
                   _fail_counter);
      _state = TrackerState::Lost;
      _fail_counter = 0;
    }
    return Lost{_p};
  }

  auto from_last_frame = track_from_last_frame(next_frame);
  if (from_last_frame) {
    _kpts1.swap(_kpts2);
    cv::swap(_desc1, _desc2);
    return Lost{_p};
  }
  auto from_last_kf = track_from_last_keyframe(next_frame);
  if (from_last_kf) {
    return Lost{_p};
  }
  auto from_map = track_from_map(next_frame);
  if (from_map) {
    return Lost{_p};
  }

  if (_fail_counter > 10) {
    spdlog::warn("Tracking failed for {} - switching to lost_tracking!",
                 _fail_counter);
    _state = TrackerState::Lost;
    _fail_counter = 0;
  }
  return Lost{_p};
}

bool Tracker::track_from_last_frame(cv::Mat const &next_frame) {
  _matcher->match(_desc1, _desc2, _matches_buf);
  spdlog::info("Number of matches: {}", _matches_buf.size());
  if (_matches_buf.size() < MIN_MATCHES) {
    spdlog::info("Not enough matches, skipping (theshold: {})", MIN_MATCHES);
    return false;
  }

  cv::Mat r_prev;
  cv::Mat t_prev;
  cv::eigen2cv(Eigen::AngleAxisd(_p.q).axis(), r_prev);
  cv::eigen2cv(_p.t, t_prev);
  std::vector<cv::Point3d> pts3d_cv;
  std::vector<cv::Point2d> pts2d_cv;

  std::vector<slammy::pose_graph::Point *> seen_pts;
  seen_pts.resize(_matches_buf.size(), nullptr);
  for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
    if (!_map_points_for_keypoint[id1]) {
      continue;
    }
    auto const &pt2 = _kpts2[id2].pt;
    pts2d_cv.emplace_back(pt2.x, pt2.y);
    auto const &t = _map_points_for_keypoint.at(id1)->t;
    pts3d_cv.emplace_back(t.x(), t.y(), t.z());
    seen_pts[id2] = _map_points_for_keypoint[id1];
  }

  if (pts2d_cv.size() < 10) {
    spdlog::info("Only {} matches were associated with 3d points",
                 pts2d_cv.size());
    return false;
  }

  auto ok = cv::solvePnP(pts3d_cv, pts2d_cv, _k_cv, {}, r_prev, t_prev, true,
                         cv::SOLVEPNP_ITERATIVE);
  if (ok) {
    Eigen::Vector3d t_eigen;
    Eigen::Vector3d r_eigen;
    cv::cv2eigen(t_prev, t_eigen);
    cv::cv2eigen(r_prev, r_eigen);

    double s = r_eigen.norm();
    Eigen::AngleAxisd aa{s, r_eigen / s};
    Eigen::Quaterniond q{aa};
    spdlog::info("aa: {} / {}", aa.angle(), Eigen::Vector3d{aa.axis()});
    spdlog::info("q: {}", q);
    spdlog::info("s: {}", s);

    _p = PoseCW{t_eigen, q};
    _map_points_for_keypoint = seen_pts; // do something with this?
  }

  return ok;
}

bool Tracker::track_from_last_keyframe(cv::Mat const &next_frame) {
  return false;
}
bool Tracker::track_from_map(cv::Mat const &next_frame) { return false; }

bool Tracker::initialise(cv::Mat const &next_frame) {
  ++_counter;
  spdlog::info("Counter: {}", _counter);

  _orb->detectAndCompute(next_frame, {}, _kpts1, _desc1);
  DBG_KPTS(next_frame, _kpts1);
  spdlog::info("Number of keypoints: {}", _kpts1.size());

  if (_kpts1.size() < MIN_KPTS) {
    spdlog::info("Not enough keypoints: skipping frame");
    return false;
  }

  if (_pg.keyframes.empty()) {
    spdlog::info("First keyframe");
    _pg.keyframes.emplace_back(std::make_unique<Keyframe>(
        next_frame, _desc1.clone(), _kpts1, PoseCW{}));
    return false;
  }

  auto const &kf1 = *_pg.keyframes.front();
  auto const &descriptors1 = kf1.descriptors;
  auto const &keypoints1 = kf1.keypoints;

  _matcher->match(descriptors1, _desc1, _matches_buf);

  spdlog::info("Number of matches: {}", _matches_buf.size());
  if (_matches_buf.size() < MIN_MATCHES) {
    spdlog::info("Not enough matches, skipping (theshold: {})", MIN_MATCHES);
    ++_fail_counter;
    if (_fail_counter == 30) {
      spdlog::info(
          "Too many matched failed with the first keyframe - restting it");
      _pg.keyframes.clear();
      _fail_counter = 0;
      return false;
    }
  }

  double cumsum_parallax = 0.0;
  for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
    auto const &pt1 = keypoints1[id1].pt;
    auto const &pt2 = _kpts1[id2].pt;
    _pts1.push_back(pt1);
    _pts2.push_back(pt2);
    auto delta = pt1 - pt2;
    cumsum_parallax += delta.x * delta.x + delta.y * delta.y;
  }

  double parallax = std::sqrt(cumsum_parallax);
  if (parallax < 10.0) {
    spdlog::info("Too little parallax between keyframes ({})", parallax);
    return false;
  }

  _p = vision::relative_pose(_pts1, _pts2, _k_cv);

  _pg.keyframes.emplace_back(
      std::make_unique<Keyframe>(next_frame, _desc1.clone(), _kpts1, _p));
  initial_triangulation();
  _state = TrackerState::Tracking;
  return true;
}

void Tracker::initial_triangulation() {
  ASS_EQ(_pg.keyframes.size(), 2);
  ASS_EQ(_pts1.size(), _pts2.size());

  auto const &p1 = _pg.keyframes[0]->pose;
  auto const &p2 = _pg.keyframes[1]->pose;

  _map_points_for_keypoint.reserve(_matches_buf.size());
  std::vector<std::pair<slammy::pose::PoseCW, Eigen::Vector2d>> observations{
      {p1, {}}, {p2, {}}};
  for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
    auto const &pt1 = _pg.keyframes[0]->keypoints[id1].pt;
    auto const &pt2 = _pg.keyframes[1]->keypoints[id2].pt;

    observations[0].second = Eigen::Vector2d{pt1.x, pt1.y};
    observations[1].second = Eigen::Vector2d{pt2.x, pt2.y};

    _map_points_for_keypoint.emplace_back(
        &_pg.add_point(slammy::vision::triangulate(observations, _k_inv))
             .seen_from(_pg.keyframes[0])
             .seen_from(_pg.keyframes[1]));
  }
}
} // namespace slammy::tracker
