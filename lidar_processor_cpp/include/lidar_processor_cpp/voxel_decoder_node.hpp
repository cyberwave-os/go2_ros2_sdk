#ifndef LIDAR_PROCESSOR_CPP__VOXEL_DECODER_NODE_HPP_
#define LIDAR_PROCESSOR_CPP__VOXEL_DECODER_NODE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "go2_interfaces/msg/voxel_map_compressed.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

extern "C" {
#include "wasm3.h"
#include "m3_env.h"
}

namespace lidar_processor_cpp
{

class VoxelWasmDecoder
{
public:
  VoxelWasmDecoder();
  ~VoxelWasmDecoder();

  bool initialize(const std::string & wasm_path);
  bool decode(
    const std::vector<uint8_t> & compressed_data,
    const std::array<double, 3> & origin,
    double resolution,
    std::vector<uint8_t> & out_positions,
    std::vector<uint8_t> & out_uvs);
  const std::string & lastError() const { return last_error_; }

private:
  uint32_t callMalloc(uint32_t size);
  int32_t readI32(uint32_t ptr) const;
  bool refreshMemory();

  IM3Environment env_{nullptr};
  IM3Runtime runtime_{nullptr};
  IM3Module module_{nullptr};
  IM3Function fn_generate_{nullptr};
  IM3Function fn_malloc_{nullptr};
  IM3Function fn_free_{nullptr};

  uint8_t * memory_{nullptr};
  uint32_t memory_size_{0};

  uint32_t input_{0};
  uint32_t decompress_buffer_{0};
  uint32_t positions_{0};
  uint32_t uvs_{0};
  uint32_t indices_{0};
  uint32_t decompressed_size_{0};
  uint32_t face_count_{0};
  uint32_t point_count_{0};
  uint32_t decompress_buffer_size_{80000};
  bool initialized_{false};
  std::string last_error_;
};

class VoxelDecoderNode : public rclcpp::Node
{
public:
  VoxelDecoderNode();

private:
  void voxelCallback(const go2_interfaces::msg::VoxelMapCompressed::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr convertToPointCloud(
    const std::vector<uint8_t> & positions,
    const std::vector<uint8_t> & uvs,
    const std::array<double, 3> & origin,
    double resolution) const;

  rclcpp::Subscription<go2_interfaces::msg::VoxelMapCompressed>::SharedPtr voxel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<VoxelWasmDecoder> decoder_;

  std::string frame_id_;
  std::string input_topic_;
  std::string output_topic_;
  double intensity_threshold_{0.0};
  bool deduplicate_points_{false};
};

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__VOXEL_DECODER_NODE_HPP_
