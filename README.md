# go2w-hesai-lidar-driver

ROS 2 driver for Hesai LiDAR sensors (PandarXT-16, PandarXT-32, PandarQT, Pandar64, Pandar40P, Pandar40M, Pandar20A, Pandar20B, PandarXTM), ported from [HesaiLidar_General_ROS](https://github.com/HesaiTechnology/HesaiLidar_General_ROS) for use on the Unitree GO2W robot.

## Background

The original Hesai driver (`HesaiLidar_General_ROS`) is ROS 1 only. This repository ports it to ROS 2, building the `HesaiLidar_General_SDK` from source for ARM (aarch64) targets such as the Jetson Orin NX.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/points_raw` | `sensor_msgs/msg/PointCloud2` | Publish | LiDAR point cloud |
| `/pandar_packets` | `hesai_lidar/msg/PandarScan` | Publish | Raw UDP packets |
| `/node_control` | `std_msgs/msg/String` | Subscribe | Send `"hesailidar_stop"` or `"all_stop"` to shutdown |

## Dependencies

- ROS 2 (Foxy or later)
- `libpcap-dev`, `libpcl-dev`, `libboost-dev`

```bash
sudo apt install libpcap-dev libpcl-dev libboost-dev
```

## Build

```bash
cd <your_colcon_workspace>/src
git clone https://github.com/koki67/go2w-hesai-lidar-driver.git
cd ..
colcon build --packages-select hesai_lidar
```

## Configuration

Edit the launch file parameters or override them at launch time:

| Parameter | Default | Description |
|---|---|---|
| `server_ip` | `192.168.123.20` | LiDAR IP address |
| `lidar_recv_port` | `2368` | UDP data port |
| `gps_port` | `10110` | GPS data port |
| `lidar_type` | `PandarXT-16` | Sensor model |
| `frame_id` | `hesai_lidar` | TF frame ID |
| `publish_type` | `both` | `"points"`, `"raw"`, or `"both"` |
| `pointcloud_reliability` | `reliable` | `/points_raw` QoS: `"reliable"` or `"best_effort"` |
| `pointcloud_qos_depth` | `1000` | Positive `/points_raw` keep-last history depth |
| `timestamp_type` | `realtime` | `""` for LiDAR time, `"realtime"` for system time |
| `pcap_file` | `""` | Path to pcap file (empty = live sensor) |
| `data_type` | `""` | `""` for live/pcap, `"rosbag"` for bag playback |
| `lidar_correction_file` | auto | Path to correction CSV |
| `coordinate_correction_flag` | `false` | Enable coordinate correction |

## Packet-order safety

The driver uses a bounded single-producer/single-consumer packet queue. It
retains 36,000 storage slots for compatibility but permits at most 400 queued
packets; packets arriving while that backlog is full are dropped instead of
blocking the receive thread.

For PandarXT, PandarXT-16, PandarXT-32, and PandarXTM packets, the trailing UDP
sequence is checked before point calculation. Forward gaps are counted, while
duplicates and backward/reordered packets are dropped. A receive idle period of
2 seconds resets the sequence baseline so a sensor reboot can recover, and
normal `uint32` sequence rollover remains valid.

Packet reception timestamps and published cloud timestamps may jitter backward
by up to 0.1 seconds. Larger regressions, including the historical approximately
7-second stale-packet replay, are dropped and reported. Queue overload, sequence,
packet timestamp, and cloud timestamp events are emitted as throttled ROS logs;
cumulative totals are logged during orderly shutdown. These checks prevent new
corrupt output but do not rewrite or repair existing bags.

Point-cloud conversion hands each completed cloud to a dedicated DDS publisher
thread through a single-slot latest-value mailbox. A slow subscriber therefore
cannot block the packet decoder or create an old-cloud backlog. If DDS remains
slower than cloud generation, the pending cloud is replaced by the newer cloud
and `cloud_queue_overwrites` records the deliberate loss.

## Run

```bash
source install/setup.bash
ros2 launch hesai_lidar hesai_lidar_launch.py
```

For high-rate multi-sensor recording, raw packet publication can be disabled
when only the point cloud is required, and `/points_raw` can use the standard
non-blocking sensor-data reliability behavior:

```bash
ros2 launch hesai_lidar hesai_lidar_launch.py \
  publish_type:=points \
  pointcloud_reliability:=best_effort \
  pointcloud_qos_depth:=5
```

The launch defaults remain `both`, `reliable`, and depth `1000` for backward
compatibility. Shutdown diagnostics report packet throughput, maximum packet
queue depth, point-cloud conversion time, DDS publish time, publish failures,
and latest-value overwrites so overloaded systems can distinguish decoder
pressure from publisher backpressure.

## License

[Apache 2.0](LICENSE) - Original driver copyright Hesai Technology.
