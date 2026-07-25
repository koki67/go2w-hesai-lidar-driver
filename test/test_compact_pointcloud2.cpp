#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "compact_pointcloud2.h"

namespace
{

template<typename T>
T readLittleEndian(const std::uint8_t * source)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), source, sizeof(T));
  if (hesai_lidar::internal::hostIsBigEndian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

PPoint point(
    float x, float y, float z, float intensity, double timestamp,
    std::uint16_t ring)
{
  PPoint result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = intensity;
  result.timestamp = timestamp;
  result.ring = ring;
  return result;
}

PPointCloud sampleCloud()
{
  PPointCloud cloud;
  cloud.header.frame_id = "hesai_lidar";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = false;
  cloud.points = {
      point(1.25F, -2.5F, 3.75F, 4.5F, 123.125, 7),
      point(-5.0F, 6.0F, -7.0F, 8.0F, 123.250, 15)};
  return cloud;
}

TEST(CompactPointCloud2, UsesPackedLosslessWireLayout)
{
  const auto message =
      hesai_lidar::internal::makeCompactPointCloud2(sampleCloud());

  EXPECT_EQ(message.header.frame_id, "hesai_lidar");
  EXPECT_EQ(message.height, 1U);
  EXPECT_EQ(message.width, 2U);
  EXPECT_FALSE(message.is_bigendian);
  EXPECT_FALSE(message.is_dense);
  EXPECT_EQ(message.point_step, 26U);
  EXPECT_EQ(message.row_step, 52U);
  EXPECT_EQ(message.data.size(), 52U);

  ASSERT_EQ(message.fields.size(), 6U);
  const std::array<std::string, 6> names{
      "x", "y", "z", "intensity", "timestamp", "ring"};
  const std::array<std::uint32_t, 6> offsets{0, 4, 8, 12, 16, 24};
  const std::array<std::uint8_t, 6> datatypes{
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT64,
      sensor_msgs::msg::PointField::UINT16};
  for (std::size_t index = 0; index < names.size(); ++index) {
    EXPECT_EQ(message.fields[index].name, names[index]);
    EXPECT_EQ(message.fields[index].offset, offsets[index]);
    EXPECT_EQ(message.fields[index].datatype, datatypes[index]);
    EXPECT_EQ(message.fields[index].count, 1U);
  }

  EXPECT_FLOAT_EQ(readLittleEndian<float>(message.data.data() + 0), 1.25F);
  EXPECT_FLOAT_EQ(readLittleEndian<float>(message.data.data() + 4), -2.5F);
  EXPECT_FLOAT_EQ(readLittleEndian<float>(message.data.data() + 8), 3.75F);
  EXPECT_FLOAT_EQ(readLittleEndian<float>(message.data.data() + 12), 4.5F);
  EXPECT_DOUBLE_EQ(readLittleEndian<double>(message.data.data() + 16), 123.125);
  EXPECT_EQ(readLittleEndian<std::uint16_t>(message.data.data() + 24), 7U);

  const auto * second = message.data.data() + message.point_step;
  EXPECT_FLOAT_EQ(readLittleEndian<float>(second + 0), -5.0F);
  EXPECT_DOUBLE_EQ(readLittleEndian<double>(second + 16), 123.250);
  EXPECT_EQ(readLittleEndian<std::uint16_t>(second + 24), 15U);
}

TEST(CompactPointCloud2, PclConsumerRestoresAlignedPointType)
{
  const auto source = sampleCloud();
  const auto message =
      hesai_lidar::internal::makeCompactPointCloud2(source);

  PPointCloud restored;
  pcl::fromROSMsg(message, restored);

  ASSERT_EQ(restored.points.size(), source.points.size());
  EXPECT_EQ(restored.header.frame_id, source.header.frame_id);
  for (std::size_t index = 0; index < source.points.size(); ++index) {
    EXPECT_FLOAT_EQ(restored.points[index].x, source.points[index].x);
    EXPECT_FLOAT_EQ(restored.points[index].y, source.points[index].y);
    EXPECT_FLOAT_EQ(restored.points[index].z, source.points[index].z);
    EXPECT_FLOAT_EQ(
        restored.points[index].intensity, source.points[index].intensity);
    EXPECT_DOUBLE_EQ(
        restored.points[index].timestamp, source.points[index].timestamp);
    EXPECT_EQ(restored.points[index].ring, source.points[index].ring);
  }
}

TEST(CompactPointCloud2, NormalizesInconsistentPclDimensions)
{
  auto cloud = sampleCloud();
  cloud.height = 0;
  cloud.width = 0;

  const auto message =
      hesai_lidar::internal::makeCompactPointCloud2(cloud);

  EXPECT_EQ(message.height, 1U);
  EXPECT_EQ(message.width, 2U);
  EXPECT_EQ(message.row_step, 52U);
  EXPECT_EQ(message.data.size(), 52U);
}

}  // namespace
