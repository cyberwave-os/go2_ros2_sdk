#include "lidar_processor_cpp/voxel_decoder_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_processor_cpp
{

namespace
{
bool fileExists(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return static_cast<bool>(in);
}

std::vector<uint8_t> readBinaryFile(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Failed to open file: " + path);
  }
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

bool linkRawImport(
  IM3Module module,
  const char * module_name,
  const char * function_name,
  const char * signature,
  M3RawCall callback)
{
  const M3Result result = m3_LinkRawFunction(module, module_name, function_name, signature, callback);
  return result == m3Err_none;
}

// Host callback for import a.a : (i32) -> i32
m3ApiRawFunction(host_adjust_memory_size)
{
  m3ApiGetArg(uint32_t, ignored);
  (void)ignored;
  m3ApiReturnType(uint32_t);
  uint32_t memory_size = 0;
  (void)m3_GetMemory(runtime, &memory_size, 0);
  m3ApiReturn(memory_size);
}

// Host callback for import a.b : (i32, i32, i32) -> void
m3ApiRawFunction(host_copy_memory_region)
{
  m3ApiGetArg(uint32_t, target);
  m3ApiGetArg(uint32_t, start);
  m3ApiGetArg(uint32_t, len);

  uint32_t memory_size = 0;
  uint8_t * memory = m3_GetMemory(runtime, &memory_size, 0);
  if (!memory) {
    m3ApiSuccess();
  }
  if (start > memory_size || target > memory_size) {
    m3ApiSuccess();
  }
  if (len > memory_size - start || len > memory_size - target) {
    m3ApiSuccess();
  }
  std::memmove(memory + target, memory + start, len);
  m3ApiSuccess();
}
}  // namespace

VoxelWasmDecoder::VoxelWasmDecoder() = default;

VoxelWasmDecoder::~VoxelWasmDecoder()
{
  if (runtime_) {
    m3_FreeRuntime(runtime_);
    runtime_ = nullptr;
  }
  if (env_) {
    m3_FreeEnvironment(env_);
    env_ = nullptr;
  }
}

bool VoxelWasmDecoder::refreshMemory()
{
  uint32_t size = 0;
  memory_ = m3_GetMemory(runtime_, &size, 0);
  memory_size_ = size;
  return memory_ != nullptr && memory_size_ > 0;
}

int32_t VoxelWasmDecoder::readI32(uint32_t ptr) const
{
  if (!memory_ || ptr + 4 > memory_size_) {
    return 0;
  }
  int32_t value = 0;
  std::memcpy(&value, memory_ + ptr, sizeof(int32_t));
  return value;
}

uint32_t VoxelWasmDecoder::callMalloc(uint32_t size)
{
  if (m3_CallV(fn_malloc_, size) != m3Err_none) {
    return 0;
  }
  uint64_t out = 0;
  if (m3_GetResultsV(fn_malloc_, &out) != m3Err_none) {
    return 0;
  }
  return static_cast<uint32_t>(out);
}

bool VoxelWasmDecoder::initialize(const std::string & wasm_path)
{
  try {
    last_error_.clear();
    auto wasm_bytes = readBinaryFile(wasm_path);

    env_ = m3_NewEnvironment();
    if (!env_) {
      last_error_ = "m3_NewEnvironment failed";
      return false;
    }
    runtime_ = m3_NewRuntime(env_, 512 * 1024, nullptr);
    if (!runtime_) {
      last_error_ = "m3_NewRuntime failed";
      return false;
    }

    M3Result result = m3_ParseModule(env_, &module_, wasm_bytes.data(), wasm_bytes.size());
    if (result != m3Err_none) {
      last_error_ = std::string("m3_ParseModule failed: ") + result;
      return false;
    }

    result = m3_LoadModule(runtime_, module_);
    if (result != m3Err_none) {
      last_error_ = std::string("m3_LoadModule failed: ") + result;
      return false;
    }

    // Required imports from wasm-objdump:
    //  - func[0] <a.a> <- a.a
    //  - func[1] <a.b> <- a.b
    bool linked_any = false;
    linked_any = linkRawImport(module_, "a", "a", "i(i)", host_adjust_memory_size) || linked_any;
    linked_any = linkRawImport(module_, "a", "b", "v(iii)", host_copy_memory_region) || linked_any;
    // Defensive fallbacks.
    linked_any = linkRawImport(module_, "env", "a", "i(i)", host_adjust_memory_size) || linked_any;
    linked_any = linkRawImport(module_, "env", "b", "v(iii)", host_copy_memory_region) || linked_any;
    linked_any = linkRawImport(module_, "", "a", "i(i)", host_adjust_memory_size) || linked_any;
    linked_any = linkRawImport(module_, "", "b", "v(iii)", host_copy_memory_region) || linked_any;
    if (!linked_any) {
      last_error_ = "Failed to link required wasm imports (a.a, a.b)";
      return false;
    }

    result = m3_FindFunction(&fn_generate_, runtime_, "e");
    if (result != m3Err_none) {
      last_error_ = std::string("m3_FindFunction(e) failed: ") + result;
      return false;
    }
    result = m3_FindFunction(&fn_malloc_, runtime_, "f");
    if (result != m3Err_none) {
      last_error_ = std::string("m3_FindFunction(f) failed: ") + result;
      return false;
    }
    (void)m3_FindFunction(&fn_free_, runtime_, "g");

    if (!refreshMemory()) {
      last_error_ = "m3_GetMemory failed";
      return false;
    }

    input_ = callMalloc(61440);
    decompress_buffer_ = callMalloc(decompress_buffer_size_);
    positions_ = callMalloc(2880000);
    uvs_ = callMalloc(1920000);
    indices_ = callMalloc(5760000);
    decompressed_size_ = callMalloc(4);
    face_count_ = callMalloc(4);
    point_count_ = callMalloc(4);
    if (
      input_ == 0 || decompress_buffer_ == 0 || positions_ == 0 || uvs_ == 0 || indices_ == 0 ||
      decompressed_size_ == 0 || face_count_ == 0 || point_count_ == 0)
    {
      last_error_ = "WASM malloc returned null for one or more buffers";
      return false;
    }

    initialized_ = true;
    return true;
  } catch (const std::exception & e) {
    last_error_ = std::string("Exception: ") + e.what();
    return false;
  } catch (...) {
    last_error_ = "Unknown exception during decoder initialization";
    return false;
  }
}

bool VoxelWasmDecoder::decode(
  const std::vector<uint8_t> & compressed_data,
  const std::array<double, 3> & origin,
  double resolution,
  std::vector<uint8_t> & out_positions,
  std::vector<uint8_t> & out_uvs)
{
  if (!initialized_ || compressed_data.empty() || resolution <= 0.0) {
    return false;
  }
  if (!refreshMemory()) {
    return false;
  }
  if (input_ + compressed_data.size() > memory_size_) {
    return false;
  }

  std::memcpy(memory_ + input_, compressed_data.data(), compressed_data.size());
  const int32_t some_v = static_cast<int32_t>(std::floor(origin[2] / resolution));

  const M3Result result = m3_CallV(
    fn_generate_,
    input_,
    static_cast<uint32_t>(compressed_data.size()),
    decompress_buffer_size_,
    decompress_buffer_,
    decompressed_size_,
    positions_,
    uvs_,
    indices_,
    face_count_,
    point_count_,
    some_v);
  if (result != m3Err_none) {
    return false;
  }

  const int32_t face_count = std::max(0, readI32(face_count_));
  const size_t pos_len = static_cast<size_t>(face_count) * 12;
  const size_t uv_len = static_cast<size_t>(face_count) * 8;
  if (positions_ + pos_len > memory_size_ || uvs_ + uv_len > memory_size_) {
    return false;
  }

  out_positions.assign(memory_ + positions_, memory_ + positions_ + pos_len);
  out_uvs.assign(memory_ + uvs_, memory_ + uvs_ + uv_len);
  return true;
}

VoxelDecoderNode::VoxelDecoderNode() : Node("voxel_decoder_node")
{
  this->declare_parameter("frame_id", "odom");
  this->declare_parameter("input_topic", "/utlidar/voxel_map_compressed");
  this->declare_parameter("output_topic", "point_cloud2");
  this->declare_parameter("intensity_threshold", 0.0);
  this->declare_parameter("deduplicate_points", false);

  frame_id_ = this->get_parameter("frame_id").as_string();
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
  deduplicate_points_ = this->get_parameter("deduplicate_points").as_bool();

  decoder_ = std::make_unique<VoxelWasmDecoder>();
  std::vector<std::string> wasm_candidates;
  try {
    const auto sdk_share_dir = ament_index_cpp::get_package_share_directory("go2_robot_sdk");
    wasm_candidates.emplace_back(sdk_share_dir + "/external_lib/libvoxel.wasm");
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Could not resolve go2_robot_sdk share dir: %s", e.what());
  }
  const char * env_wasm = std::getenv("LIBVOXEL_WASM_PATH");
  if (env_wasm && std::string(env_wasm).size() > 0) {
    wasm_candidates.emplace_back(env_wasm);
  }
  wasm_candidates.emplace_back("/ros2_ws/src/go2_robot_sdk/external_lib/libvoxel.wasm");

  bool init_ok = false;
  std::string chosen_path;
  for (const auto & candidate : wasm_candidates) {
    if (!fileExists(candidate)) {
      RCLCPP_WARN(this->get_logger(), "libvoxel.wasm not found at: %s", candidate.c_str());
      continue;
    }
    if (decoder_->initialize(candidate)) {
      init_ok = true;
      chosen_path = candidate;
      break;
    }
    RCLCPP_ERROR(this->get_logger(), "%s: %s", candidate.c_str(), decoder_->lastError().c_str());
  }
  if (!init_ok) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize VoxelWasmDecoder from any known path");
    throw std::runtime_error("Failed to initialize voxel decoder");
  }
  RCLCPP_INFO(this->get_logger(), "Loaded libvoxel.wasm from %s", chosen_path.c_str());

  auto qos = rclcpp::QoS(1).best_effort().keep_last(1);
  voxel_sub_ = this->create_subscription<go2_interfaces::msg::VoxelMapCompressed>(
    input_topic_, qos, std::bind(&VoxelDecoderNode::voxelCallback, this, std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  RCLCPP_INFO(
    this->get_logger(),
    "voxel_decoder_node ready: %s -> %s, frame_id=%s",
    input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelDecoderNode::convertToPointCloud(
  const std::vector<uint8_t> & positions,
  const std::vector<uint8_t> & uvs,
  const std::array<double, 3> & origin,
  double resolution) const
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const size_t point_count = std::min(positions.size() / 3, uvs.size() / 2);
  cloud->points.reserve(point_count);

  std::unordered_set<uint64_t> seen;
  if (deduplicate_points_) {
    seen.reserve(point_count);
  }

  for (size_t i = 0; i < point_count; ++i) {
    const float intensity = static_cast<float>(std::min(uvs[i * 2], uvs[i * 2 + 1]));
    if (intensity <= static_cast<float>(intensity_threshold_)) {
      continue;
    }

    const float x = static_cast<float>(positions[i * 3 + 0]) * static_cast<float>(resolution) +
      static_cast<float>(origin[0]);
    const float y = static_cast<float>(positions[i * 3 + 1]) * static_cast<float>(resolution) +
      static_cast<float>(origin[1]);
    const float z = static_cast<float>(positions[i * 3 + 2]) * static_cast<float>(resolution) +
      static_cast<float>(origin[2]);

    if (deduplicate_points_) {
      const int32_t qx = static_cast<int32_t>(std::round(x * 1000.0f));
      const int32_t qy = static_cast<int32_t>(std::round(y * 1000.0f));
      const int32_t qz = static_cast<int32_t>(std::round(z * 1000.0f));
      const uint64_t key =
        (static_cast<uint64_t>(static_cast<uint32_t>(qx)) << 42) ^
        (static_cast<uint64_t>(static_cast<uint32_t>(qy)) << 21) ^
        static_cast<uint64_t>(static_cast<uint32_t>(qz));
      if (!seen.insert(key).second) {
        continue;
      }
    }

    pcl::PointXYZI p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = intensity;
    cloud->points.push_back(p);
  }

  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void VoxelDecoderNode::voxelCallback(const go2_interfaces::msg::VoxelMapCompressed::SharedPtr msg)
{
  std::vector<uint8_t> decoded_positions;
  std::vector<uint8_t> decoded_uvs;
  const std::array<double, 3> origin{msg->origin[0], msg->origin[1], msg->origin[2]};
  if (!decoder_->decode(msg->data, origin, msg->resolution, decoded_positions, decoded_uvs)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Voxel decode failed");
    return;
  }

  auto cloud = convertToPointCloud(decoded_positions, decoded_uvs, origin, msg->resolution);
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = this->get_clock()->now();
  cloud_msg.header.frame_id = frame_id_;
  cloud_pub_->publish(cloud_msg);
}

}  // namespace lidar_processor_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<lidar_processor_cpp::VoxelDecoderNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "voxel_decoder_node failed: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
