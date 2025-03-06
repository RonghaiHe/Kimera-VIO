/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing RBGD + IMU data to the
 * VIO pipeline.
 * @author Antoni Rosinol
 */

#include <utility>  // for move

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"

namespace VIO {

MonoDataProviderModule::MonoDataProviderModule(
    OutputQueue* output_queue,
    const std::string& name_id,
    const bool& parallel_run,
    DataProviderInterface* data_provider) 
    : DataProviderModule(output_queue, name_id, parallel_run),
      left_frame_queue_("data_provider_left_frame_queue"),
      relative_distance_queue_("data_provider_relative_distance_queue"),
      data_provider_(data_provider),  
      cached_left_frame_(nullptr) {
  CHECK(data_provider_ != nullptr);  
  initializeRelativeDistanceCallback();
}

MonoDataProviderModule::InputUniquePtr MonoDataProviderModule::getInputPacket() {
  if (MISO::shutdown_) return nullptr;

  MonoImuSyncPacket::UniquePtr mono_imu_sync_packet = getMonoImuSyncPacket();
  if (!mono_imu_sync_packet) return nullptr;

  CHECK(vio_pipeline_callback_);
  vio_pipeline_callback_(std::move(mono_imu_sync_packet));

  return nullptr;
}

void MonoDataProviderModule::initializeRelativeDistanceCallback() {
  CHECK(data_provider_);
  data_provider_->registerRelativeDistanceCallback(
      [this](const RelativeDistanceData& data) {
        if (data.distance < 0.0 || data.distance > 100.0) {
          LOG(WARNING) << "Invalid relative distance: " << data.distance;
          return;
        }
        relative_distance_queue_.push(data);
        VLOG(3) << "Received relative distance: " << data.distance
                << " at timestamp: " << data.timestamp;
      });
}

MonoImuSyncPacket::UniquePtr MonoDataProviderModule::getMonoImuSyncPacket(
    bool cache_timestamp) {
  Frame::UniquePtr left_frame_payload = getLeftFramePayload();
  if (!left_frame_payload) return nullptr;

  const Timestamp frame_timestamp = left_frame_payload->timestamp_;
  if (timestamp_last_frame_ >= frame_timestamp) {
    LOG(WARNING) << "Dropping frame: "
                 << UtilsNumerical::NsecToSec(frame_timestamp)
                 << " (curr) <= "
                 << UtilsNumerical::NsecToSec(timestamp_last_frame_)
                 << " (last)";
    return nullptr;
  }

  ImuMeasurements imu_meas;
  switch (getTimeSyncedImuMeasurements(frame_timestamp, &imu_meas)) {
    case FrameAction::Use: break;
    case FrameAction::Wait:
      cached_left_frame_ = std::move(left_frame_payload);
      return nullptr;
    case FrameAction::Drop: return nullptr;
  }

  std::optional<double> relative_distance;
  if (!relative_distance_queue_.empty()) {
    Timestamp closest_diff = std::numeric_limits<Timestamp>::max();
    RelativeDistanceData closest_data;
    relative_distance_queue_.forEach([&](const RelativeDistanceData& data) {
      const Timestamp diff = std::abs(data.timestamp - frame_timestamp);
      if (diff < closest_diff) {
        closest_diff = diff;
        closest_data = data;
      }
      return true; 
    });

    constexpr Timestamp kMaxTimeDiff = 20'000'000; // 20ms in nanoseconds
    if (closest_diff < kMaxTimeDiff) {
      relative_distance = closest_data.distance;
      VLOG(3) << "Matched relative distance: " << *relative_distance
              << " (time diff: " << closest_diff << " ns)";
    } else {
      LOG(WARNING) << "Relative distance timestamp mismatch: " << closest_diff << " ns";
    }
  }

  auto packet = std::make_unique<MonoImuSyncPacket>(
      std::move(left_frame_payload),
      imu_meas.timestamps_,
      imu_meas.acc_gyr_
  );
  packet->relative_distance = relative_distance; 

  bool odometry_valid = false;
  gtsam::NavState external_odometry;
  if (external_odometry_buffer_) {
    ThreadsafeOdometryBuffer::QueryResult result =
        external_odometry_buffer_->getNearest(frame_timestamp, &external_odometry);
    switch (result) {
      case ThreadsafeOdometryBuffer::QueryResult::DataNotYetAvailable:
        VLOG(10) << "Odometry data not available yet";
        cached_left_frame_ = std::move(packet->frame);
        return nullptr;
      case ThreadsafeOdometryBuffer::QueryResult::DataNeverAvailable:
        odometry_valid = false;
        break;
      case ThreadsafeOdometryBuffer::QueryResult::DataAvailable:
        odometry_valid = true;
        break;
    }
  }

  if (odometry_valid) {
    packet->external_odometry = external_odometry;
  }

  if (cache_timestamp) timestamp_last_frame_ = frame_timestamp;
  return packet;
}

Frame::UniquePtr MonoDataProviderModule::getLeftFramePayload() {
  Frame::UniquePtr left_frame_payload;
  bool queue_state = false;

  if (MISO::parallel_run_) {
    queue_state = left_frame_queue_.popBlocking(left_frame_payload);
  } else {
    queue_state = left_frame_queue_.pop(left_frame_payload);
  }

  if (!queue_state) {
    LOG_IF(WARNING, MISO::parallel_run_ && !MISO::shutdown_)
        << "Queue shutdown for module: " << MISO::name_id_;
    return nullptr;
  }
  CHECK(left_frame_payload) << "Retrieved null frame from queue";

  return left_frame_payload;
}

void MonoDataProviderModule::shutdownQueues() {
  left_frame_queue_.shutdown();
  relative_distance_queue_.shutdown(); 
  DataProviderModule::shutdownQueues();
}

}  // namespace VIO
