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

#ifndef OCTOMAP_SERVER_OccupancyMapServer_H
#define OCTOMAP_SERVER_OccupancyMapServer_H

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
#include <frontend_utils/CommonStructs.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include "pcl/impl/point_types.hpp"



class OccupancyMapServer {

public:
  // typedef octomap::OcTree OcTreeT;
  typedef pcl::octree::OctreePointCloudSearch<PointF> Octree;

  OccupancyMapServer(const ros::NodeHandle &nh_ = ros::NodeHandle(),std::string m_worldFrameId="map");

  void Initialize(Octree::Ptr octree);
  void reset(const ros::NodeHandle &n,std::string frame);
  void publishProjected2DMap(const ros::Time& rostime = ros::Time::now());

protected:
  std::string m_worldFrameId;
  void update2DMap(PointF current_3dpoint, bool occupied);
  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Inserting points in a clockwise rotaion for computation of concave points
  double calculateAngle(const Eigen::Vector4f centroid, const PointF p) {
      return atan2(p.y - centroid[1], p.x - centroid[0]);
  }
  bool comparePoints(const Eigen::Vector4f  centroid, const PointF a, const PointF b) {
      double angleA = calculateAngle(centroid, a);
      double angleB = calculateAngle(centroid, b);
      return angleA > angleB; // For clockwise order
  }
  void sortPointsClockwise(std::vector<PointF>& points, Eigen::Vector4f centroid) {
      std::sort(points.begin(), points.end(), [&centroid, this](const PointF a, const PointF b) {
          return comparePoints(centroid, a, b);
      });
  }
  void insertPointClockwise(std::vector<PointF>& points, Eigen::Vector4f  centroid, const PointF newPoint) {
      auto it = std::lower_bound(points.begin(), points.end(), newPoint, [&centroid, this](const PointF a, const PointF b) {
          return comparePoints(centroid, a, b);
      });
      points.insert(it, newPoint);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void getOccupiedLimits();
  void initializeOccupancyMap();

  ros::NodeHandle nodehandle;
  ros::NodeHandle nodehandle_private;
  ros::Publisher  m_mapPub;

  Octree::Ptr m_octree;
  PointF minPt,maxPt;  
  PointF min_occupied,max_occupied;
  pcl::PointCloud<PointF>::Ptr Cloud;

  pcl::PointCloud<PointF>::Ptr filteredVoxelCloud,convexCloud,downsampledCloud;
  pcl::PointXYZ voxelpoints;
  pcl::ConcaveHull<PointF> chull;
  pcl::VoxelGrid<PointF> sor;
  pcl::PassThrough<PointF> pass;

  double m_res;

  double m_occupancyMinZ;
  double m_occupancyMaxZ;

  // downprojected 2D map:
  bool initial_check=true;  

  nav_msgs::OccupancyGrid occupancy_map;
  nav_msgs::MapMetaData oldMapInfo;
  
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  
};

#endif

// When moving window is enabled, you should change the way adjustmap is handled