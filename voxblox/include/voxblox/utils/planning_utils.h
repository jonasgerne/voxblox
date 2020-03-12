#ifndef VOXBLOX_UTILS_PLANNING_UTILS_H_
#define VOXBLOX_UTILS_PLANNING_UTILS_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
namespace utils {

/// Gets the indices of all points within the sphere.
template <typename VoxelType>
void getSphereAroundPoint(const Layer<VoxelType>& layer, const Point& center,
                          FloatingPoint radius,
                          HierarchicalIndexMap* block_voxel_list);

/**
 * Gets the indices of all points within an oriented bounding box. The bounding box is defined by it's center, the dimensions in each direction
 * and the yaw given by theta
 * @tparam VoxelType
 * @param layer
 * @param center
 * @param dim_x length along the bounding box's x-axis, center is at dim_x/2
 * @param dim_y width of the bounding box
 * @param dim_z height of the bounding box
 * @param theta yaw angle of the bounding box
 * @param block_voxel_list
 */
template <typename VoxelType>
void getOrientedBoundingBox(const Layer<VoxelType>& layer, const Point& center, const float &dim_x, const float &dim_y,
                            const float &dim_z, const float &theta, HierarchicalIndexMap *block_voxel_list);

/**
 * Gets the indices of all points within an oriented bounding box. The bounding box is defined by it's center, the dimensions in each direction
 * and the yaw given by theta
 * @tparam VoxelType
 * @param layer
 * @param transform affine transform specifying location and rotation of the bounding box
 * @param dim_x length along the bounding box's x-axis, center is at dim_x/2
 * @param dim_y width of the bounding box
 * @param dim_z height of the bounding box
 * @param block_voxel_list
 */
template <typename VoxelType>
void
getOrientedBoundingBox(const Layer<VoxelType> &layer, const Eigen::Transform<float, 3, Eigen::Affine> &transform,
                       const float &dim_x, const float &dim_y,
                       const float &dim_z, HierarchicalIndexMap *block_voxel_list);

/**
 * Gets the indices of all points around a sphere, and also allocates any
 * blocks that don't already exist.
 */
template <typename VoxelType>
void getAndAllocateSphereAroundPoint(const Point& center, FloatingPoint radius,
                                     Layer<VoxelType>* layer,
                                     HierarchicalIndexMap* block_voxel_list);

/**
 * Tools for manually editing a set of voxels. Sets the values around a sphere
 * to be artifically free or occupied, and marks them as hallucinated.
 */
template <typename VoxelType>
void fillSphereAroundPoint(const Point& center, const FloatingPoint radius,
                           const FloatingPoint max_distance_m,
                           Layer<VoxelType>* layer);
template <typename VoxelType>
void clearSphereAroundPoint(const Point& center, const FloatingPoint radius,
                            const FloatingPoint max_distance_m,
                            Layer<VoxelType>* layer);

/**
 * Utility function to get map bounds from an arbitrary layer.
 * Only accurate to block level (i.e., outer bounds of allocated blocks).
 */
template <typename VoxelType>
void computeMapBoundsFromLayer(const voxblox::Layer<VoxelType>& layer,
                               Eigen::Vector3d* lower_bound,
                               Eigen::Vector3d* upper_bound);

}  // namespace utils
}  // namespace voxblox

#include "voxblox/utils/planning_utils_inl.h"

#endif  // VOXBLOX_UTILS_PLANNING_UTILS_H_
