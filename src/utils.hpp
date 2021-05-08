#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <eigen3/Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <vector>

template <> struct fmt::formatter<Eigen::Quaterniond> : formatter<string_view> {
  template <typename FormatContext>
  auto format(Eigen::Quaterniond const &q, FormatContext &ctx) {
    return format_to(ctx.out(), "[w: {} | v: [{} {} {}]]", q.w(), q.x(), q.y(),
                     q.z());
  }
};

namespace slammy::utils {
namespace consts {
double constexpr PI = 3.141596;
}

template <class... Ts> struct variant_bullshit : Ts... { using Ts::operator()...; };
template <class... Ts> variant_bullshit(Ts...) -> variant_bullshit<Ts...>;

namespace details {
void dbg_kpts_impl(cv::Mat const &frame, char const *frame_name,
                   std::vector<cv::KeyPoint> const &kpts);

void assert_impl(bool val, char const *stringified_val, char const *fn,
                 char const *msg = nullptr);

void wait_loop();
} // namespace details
} // namespace slammy::utils

#ifdef SLAMMY_NO_DBG

#define SLAMMY_EMPTY                                                           \
  do {                                                                         \
  } while (false)

#define DBG_LOOP() SLAMMY_EMPTY
#define ASS(x) SLAMMY_EMPTY
#define ASS_EQ(x, y) SLAMMY_EMPTY
#define DBG_KPTS(img, kpts) SLAMMY_EMPTY

#else

#define DBG_LOOP() slammy::utils::details::wait_loop()

#define DBG_KPTS(img, kpts)                                                    \
  slammy::utils::details::dbg_kpts_impl(img, #img, kpts)

#define ASS(x)                                                                 \
  slammy::utils::details::assert_impl(static_cast<bool>(x), #x,                \
                                      __PRETTY_FUNCTION__)

#define ASS_EQ(x, y)                                                           \
  slammy::utils::details::assert_impl(x == y, #x " == " #y, __PRETTY_FUNCTION__)

#endif

#endif // UTILS_HPP_
