#pragma once

#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "pcl_utils.h"

namespace mfla {
/// \brief The grid map render.
class GridMapRender {
 public:
  /// \brief Construct a new Grid Map Render object
  GridMapRender();

  /// \brief Set the Resolution object
  /// \param _resolution
  void SetResolution(double _resolution);

  /// \brief Set the Map Cloud object
  /// \param _cloud
  /// \return true if success.
  void SetMapCloud(const CloudTypePtr& _cloud);

  /// \brief Save frid map.
  /// \param _file_path
  bool SaveGridMap(const std::string _file_path);

  /// \brief Convert cloud to image.
  /// \param _input_cloud
  /// \param _output_image
  /// \return true
  /// \return false
  bool Cloud2GrayImage(const CloudTypePtr& _input_cloud,
                       cv::Mat& _output_image);

  /// \brief Save yaml.
  /// \param _file_path
  /// \return true
  /// \return false
  bool SaveYaml(std::string _file_path);

 private:
  /// \brief The grid resolution
  double grid_resolution_ = 0.1;

  /// \brief The map cloud.
  CloudTypePtr cloud_;
  /// \brief The grey image.
  cv::Mat gray_image_;
  /// \brief The origin position.
  Eigen::Vector3d origin_position_ = Eigen::Vector3d::Zero();

  /// \brief The width of image.
  int image_width_ = 0;
  /// \brief The height of image.
  int image_height_ = 0;
};

}  // namespace mfla