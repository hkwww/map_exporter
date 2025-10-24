
#include "pcl_utils.h"

namespace mfla {

void DownsampleCloudAdapted(const CloudTypePtr& cloud_in,
                            CloudTypePtr& cloud_out, double leaf_size) {
  // 1. Check output data.
  if (!cloud_out) {
    cloud_out.reset(new CloudType);
  }

  cloud_out->clear();

  // 2. Compute the bounding box.
  Eigen::Vector4f min_p;
  Eigen::Vector4f max_p;
  pcl::getMinMax3D(*cloud_in, min_p, max_p);

  std::int64_t dx, dy, dz;

  double temp_dx = (max_p[0] - min_p[0]) / leaf_size;
  double temp_dy = (max_p[1] - min_p[1]) / leaf_size;
  double temp_dz = (max_p[2] - min_p[2]) / leaf_size;

  // 3. 自动切分
  Eigen::Vector3d splite_num(1, 1, 1);
  while (true) {
    dx = static_cast<std::int64_t>(temp_dx / splite_num(0)) + 1;
    dy = static_cast<std::int64_t>(temp_dy / splite_num(1)) + 1;
    dz = static_cast<std::int64_t>(temp_dz / splite_num(2)) + 1;

    if ((dx * dy * dz) <
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      break;
    }

    if (dx > dy) {
      splite_num(0) += 1;
    } else {
      splite_num(1) += 1;
    }
  }

  double stepX = (max_p[0] - min_p[0]) / splite_num(0);
  double stepY = (max_p[1] - min_p[1]) / splite_num(1);

  // 4. 分块降采样
  for (int i = 0; i < splite_num(0); i++) {
    for (int j = 0; j < splite_num(1); j++) {
      // 获得子区域中的点云
      pcl::PassThrough<PointType> pass;
      pass.setInputCloud(cloud_in);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(
          min_p[0] + i * stepX,
          min_p[0] + i * stepX + stepX);  // 保留或过滤z轴方向-1.2到0
      // pass.setFilterLimitsNegative(true);//设置过滤器限制负//设置保留范围内false
      CloudType cloud_filtered;
      pass.filter(cloud_filtered);

      pass.setInputCloud(cloud_filtered.makeShared());
      pass.setFilterFieldName("y");
      pass.setFilterLimits(
          min_p[1] + j * stepY,
          min_p[1] + j * stepY + stepY);  // 保留或过滤z轴方向-1.2到0
                                          //  VPointCloud cloud_filtered;
      pass.filter(cloud_filtered);

      pcl::VoxelGrid<PointType> sor;
      sor.setInputCloud(cloud_filtered.makeShared());
      sor.setLeafSize(leaf_size, leaf_size, leaf_size);
      CloudType cloud_downsample;
      sor.filter(cloud_downsample);

      *cloud_out += cloud_downsample;
    }
  }
}

}  // namespace mfla
