#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <string>

#include "pcl_utils.h"

namespace mfla {

/// \brief The map exporter.
class MapExporter {
 public:
  /// \brief Construct a new Map Exporter object
  MapExporter();

  /// \brief Set the Resolution.
  /// \param resolution
  void SetResolution(double resolution);

  /// \brief Load Las Cloud
  /// \param file_path
  /// \return true if success.
  bool LoadLasCloud(const std::string& file_path);

  /// \brief Load pcd cloud.
  /// \param file_path
  /// \return true if success.
  bool LoadPcdCloud(const std::string& file_path);

  /// \brief Export map to output path
  /// \param output_path
  /// \return true if success.
  bool Export(const std::string& output_path);

 protected:
  /// \brief The name of the map.
  std::string name_ = "mfla_map";

  /// \brief The map resolution.
  double map_resolution_ = 0.1;

  /// \brief The map cloud.
  CloudTypePtr map_cloud_;

  /// \brief The coordinate of the origin of the map.
  Eigen::Vector3d origin_lla_ = Eigen::Vector3d::Zero();

  /// \brief The min point of the cloud.
  Eigen::Vector3d min_point_ = Eigen::Vector3d::Zero();
};
}  // namespace mfla