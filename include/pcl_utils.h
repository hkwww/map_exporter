#pragma once

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// \brief 普通点云类型定义
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef CloudType::Ptr CloudTypePtr;

namespace mfla {

/// \brief Downsample the cloud with adaptive subdivision.
/// \param cloud_in
/// \param cloud_out
/// \param leaf_size
void DownsampleCloudAdapted(const CloudTypePtr& cloud_in,
                            CloudTypePtr& cloud_out, double leaf_size);

}  // namespace mfla