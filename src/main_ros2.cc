#include <rclcpp/rclcpp.hpp>
#include <hesai_lidar/msg/pandar_scan.hpp>
#include <hesai_lidar/msg/pandar_packet.hpp>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include <memory>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <functional>
#include "std_msgs/msg/string.hpp"
#include <boost/bind/bind.hpp>
#include "src/packet_defenses.h"
using namespace boost::placeholders;

using namespace std;
namespace hesai_lidar
{
class HesaiLidarClient: public rclcpp::Node
{
public:
  HesaiLidarClient():Node("hesai_lidar"), hsdk(nullptr)
  {
    this->declare_parameter<std::string>("pcap_file", "");
    this->declare_parameter<std::string>("server_ip", "");
    this->declare_parameter<int>("lidar_recv_port", 2368);
    this->declare_parameter<int>("gps_port", 10110);
    this->declare_parameter<double>("start_angle", 0.0);
    this->declare_parameter<std::string>("lidar_correction_file", "");
    this->declare_parameter<std::string>("lidar_type", "");
    this->declare_parameter<std::string>("frame_id", "");
    this->declare_parameter<int>("pcldata_type", 0);
    this->declare_parameter<std::string>("publish_type", "");
    this->declare_parameter<std::string>("timestamp_type", "");
    this->declare_parameter<std::string>("data_type", "");
    this->declare_parameter<std::string>("multicast_ip", "");
    this->declare_parameter<bool>("coordinate_correction_flag", false);
    this->declare_parameter<std::string>("target_frame", "");
    this->declare_parameter<std::string>("fixed_frame", "");
    this->declare_parameter<std::string>(
        "pointcloud_reliability", "reliable");
    this->declare_parameter<int>("pointcloud_qos_depth", 1000);

    std::string pointcloudReliability;
    int pointcloudQosDepth = 0;
    this->get_parameter("pointcloud_reliability", pointcloudReliability);
    this->get_parameter("pointcloud_qos_depth", pointcloudQosDepth);
    if (pointcloudQosDepth <= 0) {
      throw std::invalid_argument("pointcloud_qos_depth must be positive");
    }

    rclcpp::QoS pointcloudQos(
        rclcpp::KeepLast(static_cast<std::size_t>(pointcloudQosDepth)));
    if (pointcloudReliability == "best_effort") {
      pointcloudQos.best_effort();
    } else if (pointcloudReliability == "reliable") {
      pointcloudQos.reliable();
    } else {
      throw std::invalid_argument(
          "pointcloud_reliability must be 'reliable' or 'best_effort'");
    }

    lidarPublisher =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "points_raw", pointcloudQos);
    rclcpp::QoS packetQos(rclcpp::KeepLast(7));
    packetPublisher = this->create_publisher<hesai_lidar::msg::PandarScan>(
        "pandar_packets", packetQos);
    RCLCPP_INFO(
        this->get_logger(),
        "Point cloud QoS: reliability=%s depth=%d",
        pointcloudReliability.c_str(), pointcloudQosDepth);
    sub_node_control = this->create_subscription<std_msgs::msg::String>(
        "/node_control", 10,
        std::bind(&HesaiLidarClient::node_control_callback, this, std::placeholders::_1));

    this->initialize_sdk();
  }

  ~HesaiLidarClient() override
  {
    delete hsdk;
    hsdk = nullptr;
    RCLCPP_INFO(
        this->get_logger(),
        "Hesai cloud diagnostics: empty_cloud_drops=%llu, "
        "cloud_timestamp_regressions=%llu, clouds_published=%llu, "
        "conversion_avg_ms=%.3f, conversion_max_ms=%.3f, "
        "cloud_publish_avg_ms=%.3f, cloud_publish_max_ms=%.3f, "
        "raw_scans_published=%llu, raw_publish_avg_ms=%.3f, "
        "raw_publish_max_ms=%.3f",
        static_cast<unsigned long long>(emptyCloudDrops),
        static_cast<unsigned long long>(cloudTimestampRegressions),
        static_cast<unsigned long long>(cloudsPublished),
        averageMilliseconds(cloudConversionTotalNs, cloudsPublished),
        nanosecondsToMilliseconds(cloudConversionMaxNs),
        averageMilliseconds(cloudPublishTotalNs, cloudsPublished),
        nanosecondsToMilliseconds(cloudPublishMaxNs),
        static_cast<unsigned long long>(rawScansPublished),
        averageMilliseconds(rawPublishTotalNs, rawScansPublished),
        nanosecondsToMilliseconds(rawPublishMaxNs));
  }

private:
  static void recordDuration(
      const std::chrono::steady_clock::time_point &started,
      const std::chrono::steady_clock::time_point &finished,
      std::uint64_t &totalNanoseconds,
      std::uint64_t &maximumNanoseconds)
  {
    const auto elapsed = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            finished - started).count());
    totalNanoseconds += elapsed;
    if (elapsed > maximumNanoseconds) {
      maximumNanoseconds = elapsed;
    }
  }

  static double nanosecondsToMilliseconds(std::uint64_t nanoseconds)
  {
    return static_cast<double>(nanoseconds) / 1000000.0;
  }

  static double averageMilliseconds(
      std::uint64_t totalNanoseconds, std::uint64_t samples)
  {
    return samples == 0
        ? 0.0
        : nanosecondsToMilliseconds(totalNanoseconds) /
              static_cast<double>(samples);
  }

  void node_control_callback(const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "hesailidar_stop" || msg->data == "all_stop") {
          rclcpp::shutdown();
          std::cout << "\033[1;31m" << "HesaiLidar is forced to stop" << "\033[0m" << std::endl;
      }
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::msg::PandarScan::SharedPtr scan)
  {
    if (m_sPublishType == "both" || m_sPublishType == "points") {
      if (!cld || cld->points.empty()) {
        ++emptyCloudDrops;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping empty point cloud before publish: count=%llu",
            static_cast<unsigned long long>(emptyCloudDrops));
      } else {
        const auto timestampDecision = cloudTimestampGuard.observe(timestamp);
        if (!timestampDecision.accepted()) {
          ++cloudTimestampRegressions;
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "Dropping point cloud: timestamp event=%s previous=%.9f "
              "current=%.9f regression=%.6f sec count=%llu",
              timestampDecision.status ==
                      hesai_lidar::internal::TimestampStatus::kInvalid
                  ? "invalid"
                  : "regression",
              timestampDecision.previous, timestampDecision.current,
              timestampDecision.regression_sec,
              static_cast<unsigned long long>(cloudTimestampRegressions));
        } else {
          const auto conversionStarted = std::chrono::steady_clock::now();
          sensor_msgs::msg::PointCloud2 output;
          pcl::toROSMsg(*cld, output);
          output.header.stamp.sec = static_cast<int32_t>(timestamp);
          output.header.stamp.nanosec = static_cast<uint32_t>(
              (timestamp - output.header.stamp.sec) * 1e9);
          const auto publishStarted = std::chrono::steady_clock::now();
          lidarPublisher->publish(output);
          const auto publishFinished = std::chrono::steady_clock::now();
          recordDuration(
              conversionStarted, publishStarted, cloudConversionTotalNs,
              cloudConversionMaxNs);
          recordDuration(
              publishStarted, publishFinished, cloudPublishTotalNs,
              cloudPublishMaxNs);
          ++cloudsPublished;
#ifdef PRINT_FLAG
          std::cout.setf(ios::fixed);
          std::cout << "timestamp: " << std::setprecision(10) << timestamp << ", point size: " << cld->points.size() << std::endl;
#endif
        }
      }
    }
    if (m_sPublishType == "both" || m_sPublishType == "raw") {
      const auto publishStarted = std::chrono::steady_clock::now();
      packetPublisher->publish(*scan);
      const auto publishFinished = std::chrono::steady_clock::now();
      recordDuration(
          publishStarted, publishFinished, rawPublishTotalNs,
          rawPublishMaxNs);
      ++rawScansPublished;
#ifdef PRINT_FLAG
        std::cout << "raw size: " << scan->packets.size() << std::endl;
#endif
    }
  }

  void gpsCallback(int /*timestamp*/) {
#ifdef PRINT_FLAG
      std::cout << "gps: " << timestamp << std::endl;
#endif
  }

  void scanCallback(const hesai_lidar::msg::PandarScan::SharedPtr scan)
  {
    hsdk->PushScanPacket(scan);
  }

  void initialize_sdk()
  {
    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;

    this->get_parameter("pcap_file", pcapFile);
    this->get_parameter("server_ip", serverIp);
    this->get_parameter("lidar_recv_port", lidarRecvPort);
    this->get_parameter("gps_port", gpsPort);
    this->get_parameter("start_angle", startAngle);
    this->get_parameter("lidar_correction_file", lidarCorrectionFile);
    this->get_parameter("lidar_type", lidarType);
    this->get_parameter("frame_id", frameId);
    this->get_parameter("pcldata_type", pclDataType);
    this->get_parameter("publish_type", m_sPublishType);
    this->get_parameter("timestamp_type", m_sTimestampType);
    this->get_parameter("data_type", dataType);
    this->get_parameter("multicast_ip", multicastIp);
    this->get_parameter("coordinate_correction_flag", coordinateCorrectionFlag);
    this->get_parameter("target_frame", targetFrame);
    this->get_parameter("fixed_frame", fixedFrame);

    RCLCPP_INFO(this->get_logger(), "server_ip: %s", serverIp.c_str());

    if (!pcapFile.empty()) {
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile << " failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else {
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType) {
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = this->create_subscription<hesai_lidar::msg::PandarScan>(
            "pandar_packets", 10,
            std::bind(&HesaiLidarClient::scanCallback, this, std::placeholders::_1));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }

    if (hsdk != NULL) {
        hsdk->Start();
    } else {
        printf("create sdk fail\n");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPublisher;
  rclcpp::Publisher<hesai_lidar::msg::PandarScan>::SharedPtr packetPublisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_node_control;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  hesai_lidar::internal::TimestampGuard cloudTimestampGuard;
  std::uint64_t emptyCloudDrops{0};
  std::uint64_t cloudTimestampRegressions{0};
  std::uint64_t cloudsPublished{0};
  std::uint64_t cloudConversionTotalNs{0};
  std::uint64_t cloudConversionMaxNs{0};
  std::uint64_t cloudPublishTotalNs{0};
  std::uint64_t cloudPublishMaxNs{0};
  std::uint64_t rawScansPublished{0};
  std::uint64_t rawPublishTotalNs{0};
  std::uint64_t rawPublishMaxNs{0};
  rclcpp::Subscription<hesai_lidar::msg::PandarScan>::SharedPtr packetSubscriber;
};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hesai_lidar::HesaiLidarClient>());
  rclcpp::shutdown();
  return 0;
}
