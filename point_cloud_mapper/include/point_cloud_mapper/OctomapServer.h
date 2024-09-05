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

#include <std_srvs/Empty.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/surface/concave_hull.h>
#include <opencv2/opencv.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"



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
void update2DMap(pcl::PointXYZINormal current_3dpoint, bool occupied);
void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
float getNodeSize() {
    // Get the depth of the end node
    float resolution = m_octree->getResolution();
    float nodeSize = resolution * std::pow(2.0f, static_cast<float>(m_maxTreeDepth));
    return nodeSize;
}
double calculateAngle(const cv::Point& centroid, const cv::Point& p) {
    return atan2(p.y - centroid.y, p.x - centroid.x);
}
bool comparePoints(const cv::Point& centroid, const cv::Point& a, const cv::Point& b) {
    double angleA = calculateAngle(centroid, a);
    double angleB = calculateAngle(centroid, b);
    return angleA > angleB; // For clockwise order
}
void sortPointsClockwise(std::vector<cv::Point>& points, const cv::Point& centroid) {
    std::sort(points.begin(), points.end(), [&centroid, this](const cv::Point& a, const cv::Point& b) {
        return comparePoints(centroid, a, b);
    });
}
void insertPointClockwise(std::vector<cv::Point>& points, const cv::Point& centroid, const cv::Point& newPoint) {
    auto it = std::lower_bound(points.begin(), points.end(), newPoint, [&centroid, this](const cv::Point& a, const cv::Point& b) {
        return comparePoints(centroid, a, b);
    });
    points.insert(it, newPoint);
}
  /// hook that is called before traversing all nodes
  void handlePreNodeTraversal();
  void getOccupiedLimits();
  void initializeOccupancyMap();
  void getAllVoxelCenters(std::vector<Eigen::Vector3f>& voxelCenters);

  /// updates the downprojected 2D map as either occupied or free
  // void update2DMap(const Octree::LeafNodeIterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
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
  pcl::PointXYZINormal min_occupied,max_occupied;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr Cloud;
  std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> voxelCenters;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredVoxelCloud,convexCloud;
  pcl::PointXYZ voxelpoints;
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  
  

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
  bool initial_check=true;  

  nav_msgs::OccupancyGrid m_gridmap;
  nav_msgs::MapMetaData oldMapInfo;
  
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
};

#endif

// When moving window is enabled, you should change the way adjustmap is handled