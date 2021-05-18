#include "../src/utils.hpp"
#include "../src/vision.hpp"
#include <algorithm>
#include <fmt/ostream.h>
#include <gtest/gtest.h>
#include <iostream>
#include <spdlog/spdlog.h>

using namespace slammy::vision;
using slammy::pose::Camera;
using slammy::pose::Pose;
using slammy::pose::World;
using slammy::utils::consts::PI;

auto project(Pose<Camera, World> pcw, std::vector<Eigen::Vector3d> const &p3d,
             Eigen::Matrix3d const &k) {
  std::vector<cv::Point2f> p2d;
  p2d.reserve(p3d.size());
  for (auto const &p : p3d) {
    Eigen::Vector2d p2eigen = (k * (pcw * p)).hnormalized();
    auto asd = cv::Point2f(p2eigen.x(), p2eigen.y());
    p2d.emplace_back(asd);
  }
  return p2d;
}

TEST(triangulation, basic) {
  std::vector<Eigen::Vector3d> pts_3d = {
      {0.0, 0.0, 5.0}, {1.0, 0.0, 2.0}, {0.0, 1.0, 3.0}, {1.0, 1.0, 4.0}};

  Pose<Camera, World> p1w;
  Pose<Camera, World> p2w{Eigen::Vector3d{1.0, 0.0, 1.0}.normalized(),
                          {std::cos(PI / 4), 0.0, std::sin(PI / 4), 0.0}};

  Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d k_inv = k.inverse();
  auto pts1 = project(p1w, pts_3d, k);
  auto pts2 = project(p2w, pts_3d, k);
  std::vector<
      std::pair<slammy::pose::Pose<slammy::pose::Camera, slammy::pose::World>,
                Eigen::Vector2d>>
      observations{{p1w, {}}, {p2w, {}}};
  for (size_t i = 0; i < pts_3d.size(); ++i) {
    auto const &pt1 = pts1[i];
    auto const &pt2 = pts2[i];

    observations[0].second = Eigen::Vector2d{pt1.x, pt1.y};
    observations[1].second = Eigen::Vector2d{pt2.x, pt2.y};

    auto point = slammy::vision::triangulate(observations, k_inv);

    spdlog::info("pt: {}", point.t.transpose());
    spdlog::info("pts_3d: {}", pts_3d[i].transpose());
    EXPECT_TRUE(pts_3d[i].isApprox(point.t, 1e-3));
  }
}
