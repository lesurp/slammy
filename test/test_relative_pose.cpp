#include "../src/utils.hpp"
#include "../src/vision.hpp"
#include <algorithm>
#include <fmt/ostream.h>
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

using namespace slammy::vision;
using slammy::pose::Camera;
using slammy::pose::Pose;
using slammy::pose::World;
using slammy::utils::consts::PI;

auto project(Pose<Camera, World> pcw, std::vector<Eigen::Vector3d> const &p3d) {
  std::vector<cv::Point2f> p2d;
  p2d.resize(p3d.size());
  for (auto const &p : p3d) {
    Eigen::Vector2d p2eigen = (pcw * p).hnormalized();
    p2d.emplace_back(p2eigen.x(), p2eigen.y());
  }
  return p2d;
}

TEST(relative_pose, basic) {
  cv::Mat k = (cv::Mat_<double>(3, 3, CV_32F) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 1.0);

  std::vector<Eigen::Vector3d> pts_3d = {
      {0.0, 0.0, 5.0}, {1.0, 0.0, 2.0}, {0.0, 1.0, 3.0}, {1.0, 1.0, 4.0}};

  Pose<Camera, World> p1w;
  Pose<Camera, World> p2w{Eigen::Vector3d{1.0, 0.0, 1.0}.normalized(),
                          {std::cos(PI / 4), 0.0, std::sin(PI / 4), 0.0}};

  auto pts1 = project(p1w, pts_3d);
  auto pts2 = project(p2w, pts_3d);

  auto p2w_comp = relative_pose(pts1, pts2, k);

  spdlog::info("p2w_comp.t: {}", p2w_comp.t);
  spdlog::info("p2w_comp.q: {}", p2w_comp.q);
  spdlog::info("p2w.t: {}", p2w.t);
  spdlog::info("p2w.q: {}", p2w.q);
  EXPECT_TRUE(p2w.t.isApprox(p2w_comp.t, 1e-2));
  EXPECT_TRUE(p2w.q.isApprox(p2w_comp.q, 1e-2));
}
