/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCamera.cpp
 * @brief  test Camera
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);

namespace VIO {

class MonoCameraFixture : public ::testing::Test {
 public:
  MonoCameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        mono_camera_(nullptr),
        left_frame_queue_("left_frame_queue"),
        right_frame_queue_("right_frame_queue"),
        window_() {
    // Parse data
    parseEuroc();
    // Create Mono Camera
    mono_camera_ = VIO::make_unique<Camera>(vio_params_.camera_params_.at(0));
    CHECK(mono_camera_);
  }
  ~MonoCameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void parseEuroc() {
    // Create euroc data parser
    // Only parse one stereo frame... 0 - 1
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/V1_01_easy/", 10, 11, vio_params_);

    // Register Callbacks
    euroc_data_provider_->registerLeftFrameCallback(std::bind(
        &MonoCameraFixture::fillLeftFrameQueue, this, std::placeholders::_1));
    euroc_data_provider_->registerRightFrameCallback(std::bind(
        &MonoCameraFixture::fillRightFrameQueue, this, std::placeholders::_1));

    // Parse Euroc dataset.
    // Since we run in sequential mode, we need to spin it till it finishes.
    while (euroc_data_provider_->spin()) {
    };  // Fill queues.
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillRightFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    right_frame_queue_.push(std::move(left_frame));
  }

  /**
   * @brief compareKeypoints compares two sets of keypoints
   */
  void compareKeypoints(const KeypointsCV& kpts_1,
                        const KeypointsCV& kpts_2,
                        const float& tol) {
    ASSERT_EQ(kpts_1.size(), kpts_2.size());
    for (size_t i = 0u; i < kpts_1.size(); i++) {
      const auto& kpt_1 = kpts_1[i];
      const auto& kpt_2 = kpts_2[i];
      EXPECT_NEAR(kpt_1.x, kpt_2.x, tol);
      EXPECT_NEAR(kpt_1.y, kpt_2.y, tol);
    }
  }

  /** Visualization **/
  void drawPixelOnImg(const cv::Point2f& pixel,
                      cv::Mat& img,
                      const cv::viz::Color& color = cv::viz::Color::red(),
                      const size_t& pixel_size = 5u,
                      const uint8_t& alpha = 255u) {
    // Draw the pixel on the image
    cv::Scalar color_with_alpha =
        cv::Scalar(color[0], color[1], color[2], alpha);
    cv::circle(img, pixel, pixel_size, color_with_alpha, -1);
  }

  void drawPixelsOnImg(const std::vector<cv::Point2f>& pixels,
                       cv::Mat& img,
                       const cv::viz::Color& color = cv::viz::Color::red(),
                       const size_t& pixel_size = 5u,
                       const uint8_t& alpha = 255u) {
    // Draw the pixel on the image
    for (const auto& pixel : pixels) {
      drawPixelOnImg(pixel, img, color, pixel_size, alpha);
    }
  }

  void spinDisplay() {
    // Display 3D window
    static constexpr bool kDisplay = false;
    if (kDisplay) {
      window_.spin();
    }
  }

 protected:
  // Default Parms
  //! Params
  VioParams vio_params_;
  Camera::UniquePtr mono_camera_;

  EurocDataProvider::UniquePtr euroc_data_provider_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;

 private:
  cv::viz::Viz3d window_;
};

// Checks that the math has not been changed by accident.
TEST_F(MonoCameraFixture, BaselineCalculation) {}

TEST_F(MonoCameraFixture, project) {
  LandmarksCV lmks;
  lmks.push_back(LandmarkCV(0.0, 0.0, 1.0));
  lmks.push_back(LandmarkCV(0.0, 0.0, 2.0));
  lmks.push_back(LandmarkCV(0.0, 1.0, 2.0));
  lmks.push_back(LandmarkCV(0.0, 10.0, 20.0));
  lmks.push_back(LandmarkCV(1.0, 0.0, 2.0));

  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  // Make it easy first, use identity pose and simple intrinsics
  camera_params.body_Pose_cam_ = gtsam::Pose3::identity();
  CameraParams::Intrinsics& intrinsics = camera_params.intrinsics_;
  intrinsics.at(0) = 1.0;  // fx
  intrinsics.at(1) = 1.0;  // fy
  intrinsics.at(2) = 3.0;  // u0
  intrinsics.at(3) = 2.0;  // v0
  KeypointsCV expected_kpts;
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(KeypointCV(1.0 / 2.0 + 3.0, 2.0));

  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointsCV actual_kpts;
  EXPECT_NO_THROW(mono_camera_->project(lmks, &actual_kpts));
  compareKeypoints(expected_kpts, actual_kpts, 0.0001f);
}

TEST_F(MonoCameraFixture, projectCheirality) {
  // landmark behind camera
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  // Make it easy first, use identity pose and simple intrinsics
  camera_params.body_Pose_cam_ = gtsam::Pose3::identity();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  LandmarkCV lmk_behind_cam = LandmarkCV(0.0, 0.0, -2.0);

  KeypointCV kpt;
  EXPECT_THROW(mono_camera_->project(lmk_behind_cam, &kpt),
               gtsam::CheiralityException);
}

}  // namespace VIO
