#include <ros/ros.h>

#include <filesystem>

#include "map_exporter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_exporter_node", ros::init_options::AnonymousName);

  ros::NodeHandle priv_hh("~");
  std::string cloud_file_path;
  std::string output_path;
  priv_hh.param("cloud_file_path", cloud_file_path, std::string(""));
  priv_hh.param("output_path", output_path, std::string(""));

  std::cout << "Cloud File Path: " << cloud_file_path << std::endl;
  std::cout << "Output Path: " << output_path << std::endl;

  std::filesystem::path file_path(cloud_file_path);
  std::string ext = file_path.extension().string();

  mfla::MapExporter map_exporter;

  if (ext == ".pcd") {
    if (!map_exporter.LoadPcdCloud(cloud_file_path)) {
      std::cerr << "Failed to load pcd file: " << cloud_file_path << std::endl;
      return -1;
    }
  } else if (ext == ".las") {
    if (!map_exporter.LoadLasCloud(cloud_file_path)) {
      std::cerr << "Failed to load las file: " << cloud_file_path << std::endl;
      return -1;
    }
  } else {
    std::cerr << "Unsupported file extension: " << ext << std::endl;
    return -1;
  }

  if (!map_exporter.Export(output_path)) {
    std::cerr << "Failed to export map to: " << output_path << std::endl;
    return -1;
  }

  std::cout << "Map exported successfully to: " << output_path << std::endl;
  return 0;
}