/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <frontend_utils/CommonStructs.h>

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>


class OctomapServer {

public:
  // typedef octomap::OcTree OcTreeT;
  typedef pcl::octree::OctreePointCloudSearch<PointF> Octree;

  OctomapServer(const ros::NodeHandle &nh_ = ros::NodeHandle(),std::string m_worldFrameId="map");

  void assign_current_tree(Octree::Ptr locus_octree)
  {
    m_octree = locus_octree;
  }

  void Initialize(Octree::Ptr octree);
  void reset(const ros::NodeHandle &n,std::string frame);
  void publishProjected2DMap(const ros::Time& rostime = ros::Time::now());

protected:
  std::string m_worldFrameId;
  pcl::octree::OctreeKey genOctreeKeyforPoint(const PointF& point) {
    pcl::octree::OctreeKey key;
    float resolution = m_octree->getResolution();

    // Compute the key for each coordinate
    key.x = static_cast<unsigned int>(std::floor(point.x / resolution));
    key.y = static_cast<unsigned int>(std::floor(point.y / resolution));
    key.z = static_cast<unsigned int>(std::floor(point.z / resolution));

    return key;
}


pcl::PointXYZ octreeKeyToPoint(const pcl::octree::OctreeKey& key, float resolution) {
    // Calculate the 3D coordinates
    float x = key.x * resolution + resolution / 2.0f;
    float y = key.y * resolution + resolution / 2.0f;
    float z = key.z * resolution + resolution / 2.0f;

    // Return the 3D point
    return pcl::PointXYZ(x, y, z);
}

  
  bool coordToKeyChecked(const pcl::PointXYZINormal& point, pcl::octree::OctreeKey& key) {
    // Get the bounding box of the octree
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);

    // Check if the point is within the bounds of the octree
    if (point.x < minX || point.x > maxX ||
        point.y < minY || point.y > maxY ||
        point.z < minZ || point.z > maxZ) {
        std::cerr << "Point is out of bounds!" << std::endl;
        return false;
    }

    // Get the maximum depth of the octree
    unsigned int max_depth = m_octree->getTreeDepth();

    // Convert the coordinate to a key considering the maximum depth
    key = genOctreeKeyforPoint(point);

    // Ensure the key respects the maximum depth
    if (key.x >= (1 << max_depth) || key.y >= (1 << max_depth) || key.z >= (1 << max_depth)) {
        std::cerr << "Generated key exceeds maximum depth!" << std::endl;
        return false;
    }

    return true;
}
pcl::PointXYZ genLeafNodeCenterFromOctreeKey(const pcl::octree::OctreeKey& key) {
    pcl::PointXYZ center;
    float resolution = m_octree->getResolution();

    // Compute the center coordinates for each key
    center.x = (key.x + 0.5f) * resolution;
    center.y = (key.y + 0.5f) * resolution;
    center.z = (key.z + 0.5f) * resolution;

    return center;
}
void update2DMap(pcl::PointXYZINormal current_3dpoint, bool occupied);
float getNodeSize() {
    // Get the depth of the end node
    float resolution = m_octree->getResolution();
    float nodeSize = resolution * std::pow(2.0f, static_cast<float>(m_maxTreeDepth));
    return nodeSize;
}

  /// hook that is called before traversing all nodes
  void handlePreNodeTraversal();
  void handlePostNodeTraversal(const ros::Time& rostime, pcl::PointXYZINormal minPt,pcl::PointXYZINormal maxPt);

  /// updates the downprojected 2D map as either occupied or free
  // void update2DMap(const Octree::LeafNodeIterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const pcl::octree::OctreeKey& key) const {    
    return mapIdx((key.x - m_paddedMinKey.x) / m_multires2DScale,
                  (key.y - m_paddedMinKey.y) / m_multires2DScale);

  }

  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private;
  ros::Publisher  m_mapPub;

  Octree::Ptr m_octree;
  pcl::octree::OctreeKey m_updateBBXMin;
  pcl::octree::OctreeKey m_updateBBXMax;
  pcl::octree::OctreeKey m_paddedMinKey;
  pcl::octree::OctreeKey m_paddedMaxKey;
  pcl::PointXYZINormal minPt,maxPt;
  

  std::string m_baseFrameId; // base of the robot for ground plane filtering

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
};

#endif