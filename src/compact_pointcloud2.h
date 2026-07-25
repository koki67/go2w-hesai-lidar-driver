#ifndef HESAI_LIDAR__COMPACT_POINTCLOUD2_H_
#define HESAI_LIDAR__COMPACT_POINTCLOUD2_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "pandarGeneral/point_types.h"

namespace hesai_lidar
{
namespace internal
{

constexpr std::uint32_t kCompactPointStep = 26;

#pragma pack(push, 1)
struct CompactPointXYZIT
{
  float x;
  float y;
  float z;
  float intensity;
  double timestamp;
  std::uint16_t ring;
};
#pragma pack(pop)

static_assert(sizeof(CompactPointXYZIT) == kCompactPointStep,
    "compact PointXYZIT wire layout must remain 26 bytes");
static_assert(offsetof(CompactPointXYZIT, x) == 0, "unexpected x offset");
static_assert(offsetof(CompactPointXYZIT, y) == 4, "unexpected y offset");
static_assert(offsetof(CompactPointXYZIT, z) == 8, "unexpected z offset");
static_assert(
    offsetof(CompactPointXYZIT, intensity) == 12,
    "unexpected intensity offset");
static_assert(
    offsetof(CompactPointXYZIT, timestamp) == 16,
    "unexpected timestamp offset");
static_assert(
    offsetof(CompactPointXYZIT, ring) == 24, "unexpected ring offset");

inline bool hostIsBigEndian()
{
  const std::uint16_t marker = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01;
}

inline sensor_msgs::msg::PointField pointField(
    const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

inline sensor_msgs::msg::PointCloud2 makeCompactPointCloud2(
    const PPointCloud & cloud)
{
  using PointField = sensor_msgs::msg::PointField;

  if (cloud.points.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("point cloud exceeds PointCloud2 width range");
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header.frame_id = cloud.header.frame_id;
  output.height = cloud.height;
  output.width = cloud.width;

  const std::uint64_t declared_points =
      static_cast<std::uint64_t>(output.width) * output.height;
  if (output.height == 0 || declared_points != cloud.points.size()) {
    output.height = 1;
    output.width = static_cast<std::uint32_t>(cloud.points.size());
  }
  if (output.width >
      std::numeric_limits<std::uint32_t>::max() / kCompactPointStep) {
    throw std::length_error("point cloud row exceeds PointCloud2 row_step range");
  }

  output.fields = {
      pointField("x", 0, PointField::FLOAT32),
      pointField("y", 4, PointField::FLOAT32),
      pointField("z", 8, PointField::FLOAT32),
      pointField("intensity", 12, PointField::FLOAT32),
      pointField("timestamp", 16, PointField::FLOAT64),
      pointField("ring", 24, PointField::UINT16)};
  output.is_bigendian = hostIsBigEndian();
  output.point_step = kCompactPointStep;
  output.row_step = output.width * output.point_step;
  output.data.resize(
      static_cast<std::size_t>(output.row_step) * output.height);
  output.is_dense = cloud.is_dense;

  for (std::size_t index = 0; index < cloud.points.size(); ++index) {
    auto * destination = output.data.data() + index * kCompactPointStep;
    const auto & point = cloud.points[index];
    const CompactPointXYZIT compact{
        point.x, point.y, point.z, point.intensity, point.timestamp,
        point.ring};
    std::memcpy(destination, &compact, sizeof(compact));
  }

  return output;
}

}  // namespace internal
}  // namespace hesai_lidar

#endif  // HESAI_LIDAR__COMPACT_POINTCLOUD2_H_
