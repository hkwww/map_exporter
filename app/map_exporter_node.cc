#include <ros/ros.h>

#include "map_exporter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_exporter_node", ros::init_options::AnonymousName);

  ros::NodeHandle priv_hh("~");
  std::string las_file_path;
  std::string output_path;
  priv_hh.param("las_file_path", las_file_path, std::string(""));
  priv_hh.param("output_path", output_path, std::string(""));

  std::cout << "LAS File Path: " << las_file_path << std::endl;
  std::cout << "Output Path: " << output_path << std::endl;

  mfla::MapExporter map_exporter;
  if (!map_exporter.LoadLasCloud(las_file_path)) {
    std::cerr << "Failed to load las file: " << las_file_path << std::endl;
    return -1;
  }

  if (!map_exporter.Export(output_path)) {
    std::cerr << "Failed to export map to: " << output_path << std::endl;
    return -1;
  }

  std::cout << "Map exported successfully to: " << output_path << std::endl;
  return 0;
}