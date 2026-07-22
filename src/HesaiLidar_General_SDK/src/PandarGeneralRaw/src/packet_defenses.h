/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef SRC_PACKET_DEFENSES_H_
#define SRC_PACKET_DEFENSES_H_

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace hesai_lidar {
namespace internal {

constexpr std::size_t kXt16PacketSize = 568;
constexpr std::size_t kXtmPacketSize = 820;
constexpr std::size_t kXt32PacketSize = 1080;
constexpr std::size_t kXtSequenceSize = 4;
constexpr std::size_t kXtFactorySize = 1;
constexpr double kTimestampRegressionToleranceSec = 0.1;
constexpr double kXtSequenceRestartIdleSec = 2.0;

inline bool isSupportedXtPacketSize(std::size_t size) {
  return size == kXt16PacketSize || size == kXtmPacketSize ||
         size == kXt32PacketSize;
}

inline bool decodeXtUdpSequence(const std::uint8_t *data, std::size_t size,
                                std::uint32_t *sequence) {
  if (data == nullptr || sequence == nullptr || !isSupportedXtPacketSize(size)) {
    return false;
  }

  // The XT tail ends with the four-byte UDP sequence followed by one factory
  // byte. Decode the sequence before that trailing factory byte.
  const std::size_t offset = size - kXtFactorySize - kXtSequenceSize;
  *sequence = static_cast<std::uint32_t>(data[offset]) |
              (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
              (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
              (static_cast<std::uint32_t>(data[offset + 3]) << 24);
  return true;
}

enum class SequenceStatus {
  kFirst,
  kInOrder,
  kForwardGap,
  kDuplicate,
  kBackward,
  kRestartAfterIdle,
};

struct SequenceDecision {
  SequenceStatus status;
  std::uint32_t previous;
  std::uint32_t current;
  std::uint32_t missing;

  bool accepted() const {
    return status != SequenceStatus::kDuplicate &&
           status != SequenceStatus::kBackward;
  }
};

class XtSequenceTracker {
 public:
  explicit XtSequenceTracker(double restart_idle_sec = kXtSequenceRestartIdleSec)
      : restart_idle_sec_(restart_idle_sec) {}

  SequenceDecision observe(std::uint32_t sequence, double receive_timestamp) {
    if (!initialized_) {
      initialized_ = true;
      last_sequence_ = sequence;
      last_observed_timestamp_ = receive_timestamp;
      return {SequenceStatus::kFirst, sequence, sequence, 0};
    }

    const std::uint32_t previous = last_sequence_;
    const bool restarted_after_idle =
        std::isfinite(receive_timestamp) &&
        std::isfinite(last_observed_timestamp_) &&
        receive_timestamp - last_observed_timestamp_ >= restart_idle_sec_;
    last_observed_timestamp_ = receive_timestamp;

    if (restarted_after_idle) {
      last_sequence_ = sequence;
      return {SequenceStatus::kRestartAfterIdle, previous, sequence, 0};
    }

    const std::uint32_t delta = sequence - previous;
    if (delta == 0) {
      return {SequenceStatus::kDuplicate, previous, sequence, 0};
    }

    // Unsigned subtraction makes uint32 rollover (0xffffffff -> 0) delta 1.
    // A delta in the other half of the uint32 range is treated as backward.
    if (delta < 0x80000000u) {
      last_sequence_ = sequence;
      if (delta == 1) {
        return {SequenceStatus::kInOrder, previous, sequence, 0};
      }
      return {SequenceStatus::kForwardGap, previous, sequence, delta - 1};
    }

    return {SequenceStatus::kBackward, previous, sequence, 0};
  }

  void reset() {
    initialized_ = false;
    last_sequence_ = 0;
    last_observed_timestamp_ = 0.0;
  }

 private:
  double restart_idle_sec_;
  bool initialized_{false};
  std::uint32_t last_sequence_{0};
  double last_observed_timestamp_{0.0};
};

enum class TimestampStatus { kFirst, kAccepted, kRegression, kInvalid };

struct TimestampDecision {
  TimestampStatus status;
  double previous;
  double current;
  double regression_sec;

  bool accepted() const {
    return status == TimestampStatus::kFirst ||
           status == TimestampStatus::kAccepted;
  }
};

class TimestampGuard {
 public:
  explicit TimestampGuard(
      double tolerance_sec = kTimestampRegressionToleranceSec)
      : tolerance_sec_(tolerance_sec) {}

  TimestampDecision observe(double timestamp) {
    if (!std::isfinite(timestamp)) {
      return {TimestampStatus::kInvalid, last_timestamp_, timestamp, 0.0};
    }
    if (!initialized_) {
      initialized_ = true;
      last_timestamp_ = timestamp;
      return {TimestampStatus::kFirst, timestamp, timestamp, 0.0};
    }

    const double previous = last_timestamp_;
    const double regression = previous - timestamp;
    if (regression > tolerance_sec_) {
      return {TimestampStatus::kRegression, previous, timestamp,
              regression};
    }

    if (timestamp > last_timestamp_) {
      last_timestamp_ = timestamp;
    }
    return {TimestampStatus::kAccepted, previous, timestamp,
            regression > 0.0 ? regression : 0.0};
  }

  void reset() {
    initialized_ = false;
    last_timestamp_ = 0.0;
  }

 private:
  double tolerance_sec_;
  bool initialized_{false};
  double last_timestamp_{0.0};
};

}  // namespace internal
}  // namespace hesai_lidar

#endif  // SRC_PACKET_DEFENSES_H_
