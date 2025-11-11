#include "map_exporter.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include "grid_map_render.h"
#include "pcl_utils.h"

namespace mfla {

MapExporter::MapExporter() {}

bool MapExporter::LoadLasCloud(const std::string& file_path) {
  // 1. 创建 PDAL reader
  pdal::StageFactory factory;
  pdal::Stage* reader = factory.createStage("readers.las");
  if (!reader) {
    std::cerr << "无法创建 PDAL LAS 读取器\n";
    return false;
  }

  pdal::Options options;
  options.add("filename", file_path);
  reader->setOptions(options);

  pdal::PointTable table;
  reader->prepare(table);
  pdal::PointViewSet viewSet = reader->execute(table);
  pdal::PointViewPtr view = *viewSet.begin();

  CloudTypePtr raw_cloud(new CloudType);

  raw_cloud->resize(view->size());

  Eigen::Vector3d first_point = Eigen::Vector3d::Zero();
  for (pdal::PointId idx = 0; idx < view->size(); ++idx) {
    double point_x = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
    double point_y = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
    double point_z = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
    if (idx == 0) {
      first_point << point_x, point_y, point_z;
    }

    PointType& point = raw_cloud->points[idx];
    point.x = static_cast<float>(point_x - first_point(0));
    point.y = static_cast<float>(point_y - first_point(1));
    point.z = static_cast<float>(point_z - first_point(2));
    // if (view->hasDim(pdal::Dimension::Id::Intensity)) {
    //   point.intensity =
    //       view->getFieldAs<float>(pdal::Dimension::Id::Intensity, idx);
    // } else {
    //   point.intensity = 0.0f;
    // }
  }

  std::cout << "Loaded " << raw_cloud->size() << " points from " << file_path
            << std::endl;

  map_cloud_ = CloudTypePtr(new CloudType);
  mfla::DownsampleCloudAdapted(raw_cloud, map_cloud_, map_resolution_);

  return true;
}

bool MapExporter::LoadPcdCloud(const std::string& file_path) {
  CloudTypePtr raw_cloud(new CloudType);
  if (pcl::io::loadPCDFile(file_path, *raw_cloud) == -1) {
    std::cout << "Failed to load map cloud from " << file_path << std::endl;
    return false;
  }

  map_cloud_ = CloudTypePtr(new CloudType);
  mfla::DownsampleCloudAdapted(raw_cloud, map_cloud_, map_resolution_);

  return true;
}

bool MapExporter::Export(const std::string& output_path) {
  // 1. Check Map path.
  if (!std::filesystem::exists(output_path)) {
    if (!std::filesystem::create_directories(output_path)) {
      std::cout << "Failed to create the directory " << output_path
                << std::endl;
      return false;
    }
  }

  // 2. Save map cloud
  if (!map_cloud_) {
    std::cerr << "Map cloud is empty, please load the map cloud first.\n";
    return false;
  }
  std::string cloud_path = output_path + "/map.pcd";
  if (pcl::io::savePCDFileBinaryCompressed(cloud_path, *map_cloud_) == -1) {
    std::cerr << "Failed to save map cloud to " << cloud_path << std::endl;
    return false;
  }

  // 3. Export map information.
  YAML::Node map_node;
  map_node["map"]["name"] = name_;
  map_node["map"]["aligned"] = false;
  map_node["map"]["split"] = false;
  map_node["map"]["preload_tile"] = false;
  map_node["map"]["tile_size"] = 50;
  map_node["map"]["centralized_tile"] = false;
  map_node["map"]["origin"]["lat"] = origin_lla_(0);
  map_node["map"]["origin"]["lon"] = origin_lla_(1);
  map_node["map"]["origin"]["alt"] = origin_lla_(2);
  map_node["map"]["min_point"]["x"] = min_point_(0);
  map_node["map"]["min_point"]["y"] = min_point_(1);
  map_node["map"]["min_point"]["z"] = min_point_(2);
  map_node["map"]["scene"] = 0;

  std::string map_node_path = output_path + "/map.yaml";
  std::ofstream map_node_file(map_node_path.c_str());
  map_node_file << map_node;
  map_node_file.close();

  // 4. Export grid map
  GridMapRender grid_map_renderer;
  grid_map_renderer.SetMapCloud(map_cloud_);

  std::string grid_map_path = output_path + "/grid_map";
  grid_map_renderer.SaveGridMap(grid_map_path);

  return true;
}

}  // namespace mfla