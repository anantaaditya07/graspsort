// object_localizer_node (architecture 7.2, method amended by D-12): 2D detections + depth -> 3D
// object poses in the world frame.
//
// For every /detections message the depth image with the same stamp is taken from a small cache
// (the colour and depth images come from one Gazebo RGB-D sensor, so their stamps are equal; the
// detector copies the colour stamp). Each detection is localized with localizeBox() (D-12: wide
// depth band, table-plane footprint centre, z = table + height / 2), associated with a track by
// nearest neighbour in 3D, and averaged by the track's StabilityGate. Only stable tracks are
// published, so a mid-motion or noisy estimate never reaches the planning scene.
//
// TF: world <- image frame at the image stamp. If it fails the frame is dropped with an ERROR
// (fail loudly, SemNav D-05 rule).
//
// Subscribes:  detections (vision_msgs/Detection2DArray), camera/depth/image_raw (32FC1),
//              camera/depth/camera_info
// Publishes:   objects_3d (graspsort_msgs/ObjectPoseArray, frame world_frame)
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <graspsort_msgs/msg/object_pose_array.hpp>
#include <graspsort_perception/localizer.hpp>
#include <graspsort_perception/projection.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace graspsort_perception {

namespace gp = graspsort::perception;

namespace {
constexpr int kLogThrottleMs = 2000;
// CameraInfo K is row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1].
constexpr std::size_t kFx = 0;
constexpr std::size_t kCx = 2;
constexpr std::size_t kFy = 4;
constexpr std::size_t kCy = 5;

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Transform& t) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  out.linear() =
      Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).toRotationMatrix();
  return out;
}

std::size_t toSize(std::int64_t v, const char* name) {
  if (v < 1) {
    throw std::invalid_argument(std::string(name) + " must be >= 1");
  }
  return static_cast<std::size_t>(v);
}
}  // namespace

class ObjectLocalizerNode : public rclcpp::Node {
 public:
  ObjectLocalizerNode() : rclcpp::Node("object_localizer_node") {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");

    gp::LocalizerConfig lc;
    lc.table_height = declare_parameter<double>("table_height", lc.table_height);
    lc.depth.central_fraction =
        declare_parameter<double>("depth.central_fraction", lc.depth.central_fraction);
    lc.depth.percentile = declare_parameter<double>("depth.percentile", lc.depth.percentile);
    lc.depth.min_depth = declare_parameter<double>("depth.min_depth", lc.depth.min_depth);
    lc.depth.max_depth = declare_parameter<double>("depth.max_depth", lc.depth.max_depth);
    lc.footprint.depth_band =
        declare_parameter<double>("footprint.depth_band", lc.footprint.depth_band);
    lc.footprint.table_clearance =
        declare_parameter<double>("footprint.table_clearance", lc.footprint.table_clearance);
    lc.footprint.min_points =
        toSize(declare_parameter<std::int64_t>("footprint.min_points",
                                               static_cast<std::int64_t>(lc.footprint.min_points)),
               "footprint.min_points");
    lc.footprint.min_depth = lc.depth.min_depth;
    lc.footprint.max_depth = lc.depth.max_depth;
    config_ = lc;

    gp::TrackConfig tc;
    tc.max_association_distance =
        declare_parameter<double>("track.max_association_distance", tc.max_association_distance);
    tc.max_missed_updates =
        toSize(declare_parameter<std::int64_t>("track.max_missed_updates",
                                               static_cast<std::int64_t>(tc.max_missed_updates)),
               "track.max_missed_updates");
    tc.stability.window_size =
        toSize(declare_parameter<std::int64_t>("stability.window_size",
                                               static_cast<std::int64_t>(tc.stability.window_size)),
               "stability.window_size");
    tc.stability.min_samples =
        toSize(declare_parameter<std::int64_t>("stability.min_samples",
                                               static_cast<std::int64_t>(tc.stability.min_samples)),
               "stability.min_samples");
    tc.stability.max_position_stddev = declare_parameter<double>("stability.max_position_stddev",
                                                                 tc.stability.max_position_stddev);
    tracks_ = std::make_unique<gp::TrackAssociator>(tc);

    shapes_ = std::make_unique<gp::ShapeTable>(
        declare_parameter<std::vector<std::string>>(
            "shape_classes", std::vector<std::string>{"bottle", "sports ball"}),  // D-13
        declare_parameter<std::vector<std::int64_t>>("shape_types",
                                                     std::vector<std::int64_t>{1, 0}));

    depth_cache_size_ =
        toSize(declare_parameter<std::int64_t>("depth_cache_size", 10), "depth_cache_size");
    stamp_tolerance_ = rclcpp::Duration::from_seconds(
        declare_parameter<double>("stamp_tolerance", 0.0));  // s; 0 = exact stamp match
    tf_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("tf_timeout", 0.1));

    const auto detections_topic = declare_parameter<std::string>("detections_topic", "detections");
    const auto depth_topic =
        declare_parameter<std::string>("depth_topic", "camera/depth/image_raw");
    const auto info_topic =
        declare_parameter<std::string>("camera_info_topic", "camera/depth/camera_info");
    const auto objects_topic = declare_parameter<std::string>("objects_topic", "objects_3d");
    const auto detections_qos_depth = declare_parameter<int>("detections_qos_depth", 5);
    const auto objects_qos_depth = declare_parameter<int>("objects_qos_depth", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    objects_pub_ = create_publisher<graspsort_msgs::msg::ObjectPoseArray>(
        objects_topic, rclcpp::QoS(static_cast<std::size_t>(objects_qos_depth)));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onDepth(std::move(msg)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onInfo(*msg); });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic, rclcpp::QoS(static_cast<std::size_t>(detections_qos_depth)),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) { onDetections(*msg); });

    RCLCPP_INFO(get_logger(),
                "object_localizer_node: %s + %s -> %s (frame %s, table %.3f m, band %.3f m)",
                detections_topic.c_str(), depth_topic.c_str(), objects_topic.c_str(),
                world_frame_.c_str(), config_.table_height, config_.footprint.depth_band);
  }

 private:
  void onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    depth_cache_.push_back(std::move(msg));
    while (depth_cache_.size() > depth_cache_size_) {
      depth_cache_.pop_front();
    }
  }

  void onInfo(const sensor_msgs::msg::CameraInfo& msg) {
    intrinsics_ = gp::CameraIntrinsics{msg.k[kFx], msg.k[kFy], msg.k[kCx], msg.k[kCy]};
  }

  sensor_msgs::msg::Image::ConstSharedPtr findDepth(const rclcpp::Time& stamp) const {
    sensor_msgs::msg::Image::ConstSharedPtr best;
    std::int64_t best_dt_ns = stamp_tolerance_.nanoseconds();
    for (const auto& d : depth_cache_) {
      const std::int64_t dt_ns = std::llabs((rclcpp::Time(d->header.stamp) - stamp).nanoseconds());
      if (dt_ns <= best_dt_ns) {
        best = d;
        best_dt_ns = dt_ns;
      }
    }
    return best;
  }

  void onDetections(const vision_msgs::msg::Detection2DArray& msg) {
    if (!intrinsics_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "waiting for camera_info");
      return;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    const auto depth_msg = findDepth(stamp);
    if (!depth_msg) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                           "no depth image within %.3f s of detections stamp %.3f",
                           stamp_tolerance_.seconds(), stamp.seconds());
      return;
    }
    if (depth_msg->header.frame_id != msg.header.frame_id) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                            "depth frame '%s' differs from detections frame '%s'",
                            depth_msg->header.frame_id.c_str(), msg.header.frame_id.c_str());
      return;
    }

    Eigen::Isometry3d world_T_optical;
    try {
      world_T_optical =
          toEigen(tf_buffer_->lookupTransform(world_frame_, msg.header.frame_id, stamp, tf_timeout_)
                      .transform);
    } catch (const tf2::TransformException& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                            "TF %s <- %s at %.3f failed: %s", world_frame_.c_str(),
                            msg.header.frame_id.c_str(), stamp.seconds(), e.what());
      return;
    }

    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "cv_bridge: %s", e.what());
      return;
    }

    std::vector<gp::ObjectMeasurement> measurements;
    for (const auto& det : msg.detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto& hyp = det.results.front().hypothesis;
      const auto shape = shapes_->find(hyp.class_id);
      if (!shape) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                             "class '%s' has no shape in shape_classes; ignored",
                             hyp.class_id.c_str());
        continue;
      }
      const cv::Rect box = gp::boxFromCenterSize(
          det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.size_x, det.bbox.size_y);
      std::optional<gp::ObjectSample> sample;
      try {
        sample =
            gp::localizeBox(depth->image, box, *intrinsics_, world_T_optical, config_, hyp.score);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "%s (encoding %s)",
                              e.what(), depth_msg->encoding.c_str());
        return;
      }
      if (sample) {
        measurements.push_back({hyp.class_id, gp::applyShape(*sample, *shape)});
      }
    }
    tracks_->update(measurements);
    publish(msg.header.stamp);
  }

  void publish(const builtin_interfaces::msg::Time& stamp) {
    graspsort_msgs::msg::ObjectPoseArray out;
    out.header.stamp = stamp;
    out.header.frame_id = world_frame_;
    for (const auto& track : tracks_->tracks()) {
      const auto est = track.gate.stableEstimate();
      if (!est) {
        continue;
      }
      graspsort_msgs::msg::ObjectPose o;
      o.header = out.header;
      o.id = track.id;
      o.class_name = track.class_name;
      o.confidence = static_cast<float>(est->confidence);
      o.pose.position.x = est->position.x();
      o.pose.position.y = est->position.y();
      o.pose.position.z = est->position.z();
      const Eigen::Quaterniond q(Eigen::AngleAxisd(est->yaw, Eigen::Vector3d::UnitZ()));
      o.pose.orientation.x = q.x();
      o.pose.orientation.y = q.y();
      o.pose.orientation.z = q.z();
      o.pose.orientation.w = q.w();
      o.size.x = est->size_x;
      o.size.y = est->size_y;
      o.size.z = est->height;
      o.shape = static_cast<std::uint8_t>(*shapes_->find(track.class_name));
      out.objects.push_back(o);
    }
    objects_pub_->publish(out);
  }

  std::string world_frame_;
  gp::LocalizerConfig config_;
  std::unique_ptr<gp::TrackAssociator> tracks_;
  std::unique_ptr<gp::ShapeTable> shapes_;
  std::size_t depth_cache_size_{1};
  rclcpp::Duration stamp_tolerance_{0, 0};
  rclcpp::Duration tf_timeout_{0, 0};
  std::optional<gp::CameraIntrinsics> intrinsics_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depth_cache_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<graspsort_msgs::msg::ObjectPoseArray>::SharedPtr objects_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
};

}  // namespace graspsort_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<graspsort_perception::ObjectLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}
