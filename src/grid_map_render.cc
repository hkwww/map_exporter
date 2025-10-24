#include "grid_map_render.h"

#include <filesystem>

namespace mfla {

GridMapRender::GridMapRender() = default;

void GridMapRender::SetResolution(double _resolution) {
  grid_resolution_ = _resolution;
}

void GridMapRender::SetMapCloud(const CloudTypePtr& _cloud) { cloud_ = _cloud; }

bool GridMapRender::SaveGridMap(const std::string _file_path) {
  // 1. Create directory
  if (!std::filesystem::exists(_file_path)) {
    if (!std::filesystem::create_directories(_file_path)) {
      std::cout << "Failed to create the directory " << _file_path << std::endl;
      return false;
    }
  }

  // 2. Cloud to gray image
  if (!Cloud2GrayImage(cloud_, gray_image_)) {
    return false;
  }

  // 3. Save
  std::string image_png_name = _file_path + "/map_image.png";
  std::string image_pgm_name = _file_path + "/map_image.pgm";

  cv::imwrite(image_png_name, gray_image_);
  cv::imwrite(image_pgm_name, gray_image_);

  SaveYaml(_file_path);

  return true;
}

bool GridMapRender::SaveYaml(std::string _file_path) {
  std::string yaml_path = _file_path + "/map.yaml";
  FILE* yaml = fopen(yaml_path.c_str(), "w");
  std::string gray_image_local_name = "map_image.pgm";

  fprintf(yaml,
          "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
          "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          gray_image_local_name.c_str(), grid_resolution_, origin_position_(0),
          origin_position_(1), origin_position_(2));

  fclose(yaml);

  return true;
}

bool GridMapRender::Cloud2GrayImage(const CloudTypePtr& _input_cloud,
                                    cv::Mat& _output_image) {
  // 1. Check cloud.
  if (!_input_cloud || _input_cloud->empty()) {
    std::cout << "The cloud is nullptr, please set the cloud first."
              << std::endl;
    return false;
  }

  // 2. Convert cloud to image
  Eigen::Vector4f min_cloud;
  Eigen::Vector4f max_cloud;
  _input_cloud->is_dense = false;

  pcl::getMinMax3D<PointType>(*_input_cloud, min_cloud, max_cloud);
  min_cloud(0) -= 10;
  min_cloud(1) -= 10;
  max_cloud(0) += 10;
  max_cloud(1) += 10;
  float len_x = max_cloud(0) - min_cloud(0);
  float len_y = max_cloud(1) - min_cloud(1);

  origin_position_ = Eigen::Vector3d(min_cloud(0), min_cloud(1), 0);

  // x轴方向是width y方向是height
  int width = static_cast<int>(len_x / grid_resolution_);
  int height = static_cast<int>(len_y / grid_resolution_);

  _output_image.create(height, width, CV_8UC1);
  _output_image.setTo(cv::Scalar(255));

  for (const auto& point : _input_cloud->points) {
    int index_y = static_cast<int>((point.x - min_cloud(0)) / grid_resolution_);
    int index_x =
        height - static_cast<int>((point.y - min_cloud(1)) / grid_resolution_);

    if (index_y < 0 || index_y >= width || index_x < 0 || index_x >= height) {
      continue;
    }

    if (static_cast<int>(_output_image.at<uchar>(index_x, index_y)) >= 20) {
      _output_image.at<uchar>(index_x, index_y) -= 20;
    }
  }

  cv::normalize(_output_image, _output_image, 0, 255, cv::NORM_MINMAX);

  image_width_ = width;
  image_height_ = height;

  return true;
}

}  // namespace mfla