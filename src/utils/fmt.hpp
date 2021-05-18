#ifndef FMT_HPP_
#define FMT_HPP_

#include "../magic_enum.hpp"
#include "../pose.hpp"
#include <eigen3/Eigen/Geometry>
#include <fmt/format.h>

template <> struct fmt::formatter<Eigen::Quaterniond> : formatter<string_view> {
  template <typename FormatContext>
  auto format(Eigen::Quaterniond const &q, FormatContext &ctx) {
    return format_to(ctx.out(), "(w: {} | v: ({} {} {}))", q.w(), q.x(), q.y(),
                     q.z());
  }
};

template <> struct fmt::formatter<Eigen::Vector3d> : formatter<string_view> {
  template <typename FormatContext>
  auto format(Eigen::Vector3d const &v, FormatContext &ctx) {
    return format_to(ctx.out(), "({} {} {})", v.x(), v.y(), v.z());
  }
};

template <slammy::pose::CoordinateFrame To, slammy::pose::CoordinateFrame From>
struct fmt::formatter<slammy::pose::Pose<To, From>> : formatter<string_view> {
  template <typename FormatContext>
  auto format(slammy::pose::Pose<To, From> const &p, FormatContext &ctx) {
    return format_to(ctx.out(), "Pose {{{}, {}}}: [ {} | {} ]",
                     magic_enum::enum_name(To), magic_enum::enum_name(From),
                     p.q, p.t);
  }
};

#endif // FMT_HPP_
