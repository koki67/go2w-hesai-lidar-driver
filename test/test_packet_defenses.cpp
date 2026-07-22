#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "packet_defenses.h"

namespace {

std::vector<std::uint8_t> packetWithSequence(std::size_t size,
                                             std::uint32_t sequence,
                                             std::uint8_t factory = 0xa5u) {
  std::vector<std::uint8_t> packet(size, 0);
  const std::size_t offset = size - hesai_lidar::internal::kXtFactorySize -
                             hesai_lidar::internal::kXtSequenceSize;
  packet[offset] = static_cast<std::uint8_t>(sequence & 0xffu);
  packet[offset + 1] = static_cast<std::uint8_t>((sequence >> 8) & 0xffu);
  packet[offset + 2] = static_cast<std::uint8_t>((sequence >> 16) & 0xffu);
  packet[offset + 3] = static_cast<std::uint8_t>((sequence >> 24) & 0xffu);
  packet[size - 1] = factory;
  return packet;
}

}  // namespace

TEST(XtUdpSequence, DecodesAllSupportedPacketSizes) {
  const std::size_t sizes[] = {
      hesai_lidar::internal::kXt16PacketSize,
      hesai_lidar::internal::kXtmPacketSize,
      hesai_lidar::internal::kXt32PacketSize,
  };

  for (const std::size_t size : sizes) {
    const auto packet = packetWithSequence(size, 0x78563412u);
    std::uint32_t decoded = 0;
    ASSERT_TRUE(hesai_lidar::internal::decodeXtUdpSequence(
        packet.data(), packet.size(), &decoded));
    EXPECT_EQ(decoded, 0x78563412u);
  }

  const auto unsupported = packetWithSequence(100, 10);
  std::uint32_t decoded = 0;
  EXPECT_FALSE(hesai_lidar::internal::decodeXtUdpSequence(
      unsupported.data(), unsupported.size(), &decoded));
}

TEST(XtUdpSequence, IgnoresTrailingFactoryByte) {
  const auto first = packetWithSequence(
      hesai_lidar::internal::kXt16PacketSize, 0x04030201u, 0x00u);
  const auto second = packetWithSequence(
      hesai_lidar::internal::kXt16PacketSize, 0x04030201u, 0xffu);
  std::uint32_t first_decoded = 0;
  std::uint32_t second_decoded = 0;

  ASSERT_TRUE(hesai_lidar::internal::decodeXtUdpSequence(
      first.data(), first.size(), &first_decoded));
  ASSERT_TRUE(hesai_lidar::internal::decodeXtUdpSequence(
      second.data(), second.size(), &second_decoded));
  EXPECT_EQ(first_decoded, 0x04030201u);
  EXPECT_EQ(second_decoded, 0x04030201u);
}

TEST(XtSequenceTracker, HandlesNormalGapDuplicateAndBackwardPackets) {
  hesai_lidar::internal::XtSequenceTracker tracker;

  EXPECT_EQ(tracker.observe(10, 1.0).status,
            hesai_lidar::internal::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(11, 1.1).status,
            hesai_lidar::internal::SequenceStatus::kInOrder);

  const auto gap = tracker.observe(15, 1.2);
  EXPECT_EQ(gap.status, hesai_lidar::internal::SequenceStatus::kForwardGap);
  EXPECT_EQ(gap.missing, 3u);
  EXPECT_TRUE(gap.accepted());

  const auto duplicate = tracker.observe(15, 1.3);
  EXPECT_EQ(duplicate.status,
            hesai_lidar::internal::SequenceStatus::kDuplicate);
  EXPECT_FALSE(duplicate.accepted());

  const auto backward = tracker.observe(14, 1.4);
  EXPECT_EQ(backward.status,
            hesai_lidar::internal::SequenceStatus::kBackward);
  EXPECT_FALSE(backward.accepted());
}

TEST(XtSequenceTracker, AcceptsUint32Rollover) {
  hesai_lidar::internal::XtSequenceTracker tracker;

  EXPECT_EQ(tracker.observe(0xffffffffu, 1.0).status,
            hesai_lidar::internal::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(0u, 1.1).status,
            hesai_lidar::internal::SequenceStatus::kInOrder);
  EXPECT_EQ(tracker.observe(1u, 1.2).status,
            hesai_lidar::internal::SequenceStatus::kInOrder);
}

TEST(XtSequenceTracker, RecoversFromRestartAfterReceiveIdle) {
  hesai_lidar::internal::XtSequenceTracker tracker;

  EXPECT_EQ(tracker.observe(1000, 10.0).status,
            hesai_lidar::internal::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(5, 10.1).status,
            hesai_lidar::internal::SequenceStatus::kBackward);
  const auto restarted = tracker.observe(
      5, 10.1 + hesai_lidar::internal::kXtSequenceRestartIdleSec);
  EXPECT_EQ(restarted.status,
            hesai_lidar::internal::SequenceStatus::kRestartAfterIdle);
  EXPECT_TRUE(restarted.accepted());
  EXPECT_EQ(tracker.observe(6, 12.2).status,
            hesai_lidar::internal::SequenceStatus::kInOrder);
}

TEST(TimestampGuard, AcceptsMonotonicInputAndSmallJitter) {
  hesai_lidar::internal::TimestampGuard guard;

  EXPECT_EQ(guard.observe(100.0).status,
            hesai_lidar::internal::TimestampStatus::kFirst);
  EXPECT_EQ(guard.observe(101.0).status,
            hesai_lidar::internal::TimestampStatus::kAccepted);
  const auto jittered = guard.observe(
      101.0 - hesai_lidar::internal::kTimestampRegressionToleranceSec / 2.0);
  EXPECT_TRUE(jittered.accepted());
}

TEST(TimestampGuard, RejectsSevenSecondRegressionAndRecovers) {
  hesai_lidar::internal::TimestampGuard guard;

  ASSERT_TRUE(guard.observe(100.0).accepted());
  ASSERT_TRUE(guard.observe(101.0).accepted());
  const auto stale = guard.observe(94.0);
  EXPECT_EQ(stale.status,
            hesai_lidar::internal::TimestampStatus::kRegression);
  EXPECT_NEAR(stale.regression_sec, 7.0, 1e-12);
  EXPECT_FALSE(stale.accepted());
  EXPECT_TRUE(guard.observe(102.0).accepted());

  guard.reset();
  EXPECT_EQ(guard.observe(50.0).status,
            hesai_lidar::internal::TimestampStatus::kFirst);
}

TEST(TimestampGuard, RejectsInvalidTimestampAtCloudBoundary) {
  hesai_lidar::internal::TimestampGuard cloud_guard;

  ASSERT_TRUE(cloud_guard.observe(1000.0).accepted());
  const auto invalid =
      cloud_guard.observe(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(invalid.status,
            hesai_lidar::internal::TimestampStatus::kInvalid);
  EXPECT_FALSE(invalid.accepted());
}

TEST(TimestampGuard, RejectsCloudRegressionBeyondTolerance) {
  hesai_lidar::internal::TimestampGuard cloud_guard;

  ASSERT_TRUE(cloud_guard.observe(1000.0).accepted());
  const auto stale_cloud = cloud_guard.observe(992.835);
  EXPECT_EQ(stale_cloud.status,
            hesai_lidar::internal::TimestampStatus::kRegression);
  EXPECT_NEAR(stale_cloud.regression_sec, 7.165, 1e-12);
  EXPECT_FALSE(stale_cloud.accepted());
}
