// YoloDetector (ORT-backed, ROS-free) against the real models/yolov8n.onnx.
#include <gtest/gtest.h>

#include <graspsort_perception/yolo_detector.hpp>
#include <opencv2/imgproc.hpp>

#ifndef GRASPSORT_TEST_MODEL_PATH
#error "GRASPSORT_TEST_MODEL_PATH must be defined"
#endif

using graspsort_perception::StageTiming;
using graspsort_perception::YoloDetector;
using graspsort_perception::YoloOptions;

namespace {
constexpr int kThreads = 1;
constexpr float kConf = 0.35F;
constexpr float kIou = 0.45F;
}  // namespace

TEST(YoloDetector, LoadsModelGeometryAndNames) {
  YoloDetector det(YoloOptions{GRASPSORT_TEST_MODEL_PATH, kThreads, false});
  EXPECT_EQ(det.input_size(), 640);
  ASSERT_EQ(det.class_names().size(), 80U);
  EXPECT_EQ(det.class_names()[0], "person");
  EXPECT_EQ(det.class_names()[56], "chair");
  EXPECT_FALSE(det.cuda_active());
}

TEST(YoloDetector, BlankFrameGivesNoDetectionsAndTimings) {
  YoloDetector det(YoloOptions{GRASPSORT_TEST_MODEL_PATH, kThreads, false});
  const cv::Mat grey(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
  StageTiming t{-1.0, -1.0, -1.0};
  const auto dets = det.detect(grey, kConf, kIou, nullptr, &t);
  EXPECT_TRUE(dets.empty());
  EXPECT_GE(t.preprocess_ms, 0.0);
  EXPECT_GT(t.infer_ms, 0.0);
  EXPECT_GE(t.postprocess_ms, 0.0);
  // Second call reuses the pre-allocated input buffer and must give the same result.
  EXPECT_TRUE(det.detect(grey, kConf, kIou, nullptr, &t).empty());
}

TEST(YoloDetector, UseCudaOnCpuBuildFallsBackWithWarning) {
  YoloDetector det(YoloOptions{GRASPSORT_TEST_MODEL_PATH, kThreads, true});
  EXPECT_FALSE(det.cuda_active());
  EXPECT_FALSE(det.warnings().empty());
}
