#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#define SPDLOG_FMT_EXTERNAL
#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include <algorithm>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <vector>

// TODO file-ify
auto constexpr MIN_KPTS = 500;
auto constexpr MIN_MATCHES = 100;

#define DBG_LOOP()                                                             \
  do {                                                                         \
    cv::waitKey(1);                                                            \
  } while (false)

#define DBG_KPTS(img, kpts)                                                    \
  do {                                                                         \
    dbg_kpts_impl(img, #img, kpts);                                            \
  } while (false)

void dbg_kpts_impl(cv::Mat const &frame, char const *frame_name,
                   std::vector<cv::KeyPoint> const &kpts) {
  auto f = frame.clone();
  for (auto const &kp : kpts) {
    cv::circle(f, kp.pt, 2, cv::Scalar(0, 255, 0));
  }
  cv::imshow(frame_name, f);
}

#define ASSERT(x)                                                              \
  do {                                                                         \
    assert_impl(static_cast<bool>(x), #x, __PRETTY_FUNCTION__);                \
  } while (false)
#define ASSERT_EQ(x, y)                                                        \
  do {                                                                         \
    assert_impl(x == y, #x " == " #y, __PRETTY_FUNCTION__);                    \
  } while (false)

void assert_impl(bool val, char const *stringified_val, char const *fn,
                 char const *msg = nullptr) {
  if (val) {
    return;
  }

  spdlog::critical("Assertion `{}` failed at {}", stringified_val, fn);
  if (msg) {
    spdlog::critical("Additional information: {}", msg);
  }
}

struct Pose {
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
};

Pose operator*(Pose const &p_ab, Pose const &p_bc) {
  Pose p_ac;
  p_ac.q = p_ab.q * p_bc.q;
  p_ac.t = p_ab.t + p_ab.q._transformVector(p_bc.t);
  return p_ac;
}

struct Keyframe {
  cv::Mat frame;
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  Pose pose;

  Keyframe(cv::Mat const frame, cv::Mat const descriptors,
           std::vector<cv::KeyPoint> const keypoints, Pose const &pose)
      : frame(frame), descriptors(descriptors), keypoints(keypoints),
        pose(pose) {}
};

class Tracker {
public:
  Tracker(cv::Mat const &k) : _k(k), _orb(cv::ORB::create()), _matcher(cv::BFMatcher::create()) {}
  void next(cv::Mat const &next_frame) {
    spdlog::info("Tracker.next");
    spdlog::info("Number channels: {}", next_frame.channels());
    _orb->detectAndCompute(next_frame, {}, _kpts_buf, _desc_buf);
    DBG_KPTS(next_frame, _kpts_buf);
    spdlog::info("Number of keypoints: {}", _kpts_buf.size());
    if (_keyframes.empty()) {
      spdlog::info("No keyframe stored - saving as first one");
      _counter = 0;
      _keyframes.emplace_back(next_frame, _desc_buf.clone(),
                              std::vector<cv::KeyPoint>{}, Pose{});
      return;
    }
    ++_counter;
    spdlog::info("Counter: {}", _counter);
    if (_counter % 10 != 0) {
      spdlog::info("Skipping frame");
      return;
    }
    if (_kpts_buf.size() < MIN_KPTS) {
      spdlog::info("Not enough keypoints: skipping frame");
      if (_counter_failed_orb == 5) {
      }
      return;
    }
    _keyframes.emplace_back(next_frame, _desc_buf.clone(),
                            std::vector<cv::KeyPoint>{}, Pose{});
    spdlog::info("Number of keyframes: {}", _keyframes.size());

    if (_keyframes.size() == 5) {
      spdlog::info("Initializing map");
      _init_map();
    }
  }

  void _init_map() {
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    spdlog::info("_init_map");
    for (int i = 1; i < _keyframes.size(); ++i) {
      auto const &[frame1, descriptors1, keypoints1, p_w1] = _keyframes[i - 1];
      auto &[frame2, descriptors2, keypoints2, p_w2] = _keyframes[i];

      _matcher->match(descriptors1, descriptors2, _matches_buf);
      spdlog::info("Number of matches: {}", _matches_buf.size());

      pts1.clear();
      pts2.clear();
      if (_matches_buf.size() < MIN_MATCHES) {
        spdlog::warn("Not enough matches between kfs: resetting map");
        _keyframes.clear();
        return;
      }
      for (auto const &[id1, id2, _id, _dist] : _matches_buf) {
        pts1.push_back(keypoints1[id1].pt);
        pts2.push_back(keypoints2[id2].pt);
      }

      auto e = cv::findEssentialMat(pts1, pts2);

      cv::Mat t;
      cv::Mat r;
      cv::recoverPose(e, pts1, pts2, _k, r, t);

      auto p_12 = Pose{};
      Eigen::Matrix3d r_eigen;
      cv::cv2eigen(t, p_12.t);
      cv::cv2eigen(r, r_eigen);
      p_12.q = Eigen::Quaterniond{r_eigen};

      // TODO: obv doesn't work bc of scale!
      p_w2 = p_w1 * p_12;
    }
  }

  std::vector<Keyframe> const &kfs() const { return _keyframes; }

private:
  unsigned int _counter = 0;
  unsigned int _counter_failed_orb = 0;
  cv::Ptr<cv::ORB> _orb;
  cv::Ptr<cv::BFMatcher> _matcher;
  std::vector<cv::KeyPoint> _kpts_buf;
  cv::Mat _desc_buf;
  std::vector<cv::DMatch> _matches_buf;
  std::vector<Keyframe> _keyframes;
  cv::Mat _k;
};

class CameraSubscriber {
public:
  CameraSubscriber(ros::NodeHandle &n, std::string const &topic,
                   uint32_t queue_size)
      : _sub(n.subscribe(topic, queue_size, CameraSubscriber::on_next_frame)) {}

  cv_bridge::CvImagePtr next_frame() {
    spdlog::info("Waiting for next frame");
    auto l = std::unique_lock{_last_image_mutex};
    _last_image_cv.wait(l, [] { return _last_image.get(); });
    auto asd = std::move(_last_image);
    ASSERT(asd);
    ASSERT_EQ(_last_image, nullptr);
    spdlog::info("Got it");
    return std::move(asd);
  }

private:
  static cv_bridge::CvImagePtr _last_image;
  static std::mutex _last_image_mutex;
  static std::condition_variable _last_image_cv;
  ros::Subscriber _sub;

  static void on_next_frame(sensor_msgs::Image const &image) {
    spdlog::info("on_next_frame called");
    {
      auto l = std::lock_guard{_last_image_mutex};
      _last_image =
          cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    _last_image_cv.notify_one();
  }
};

cv_bridge::CvImagePtr CameraSubscriber::_last_image = {};
std::mutex CameraSubscriber::_last_image_mutex = {};
std::condition_variable CameraSubscriber::_last_image_cv = {};

int main(int argc, char **argv) {
  ros::init(argc, argv, "slammy");
  ros::NodeHandle n;

  auto path_displayer = n.advertise<nav_msgs::Path>("slammy_pose", 5);
  auto camera = CameraSubscriber{n, "/camera", 1000};
  spdlog::info("Subscribed to {}", "/camera");

  auto last_pose = geometry_msgs::PoseStamped();
  last_pose.header.frame_id = "slammy_pose";
  last_pose.header.seq = 0;

  auto tracker = Tracker{cv::Mat{}};
  auto path = nav_msgs::Path{};
  path.header.frame_id = "slammy_pose";
  path.header.seq = 0;

  auto tracker_thread = std::thread([&] {
    while (ros::ok()) {
      auto next_frame = camera.next_frame();
      spdlog::info("Got next frame");

      tracker.next(next_frame->image);

      path.poses.clear();
      for (auto const &kf : tracker.kfs()) {
        auto const &[t, q] = kf.pose;
        last_pose.pose.orientation.w = q.w();
        last_pose.pose.orientation.x = q.x();
        last_pose.pose.orientation.y = q.y();
        last_pose.pose.orientation.z = q.z();
        last_pose.pose.position.x = t.x();
        last_pose.pose.position.y = t.y();
        last_pose.pose.position.z = t.z();
        path.poses.emplace_back(last_pose);
      }
      path_displayer.publish(path);
      DBG_LOOP();
    }
  });

  ros::spin();

  tracker_thread.join();
  return 0;
}
