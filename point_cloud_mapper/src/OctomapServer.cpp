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

#include <point_cloud_mapper/OctomapServer.h>
#include <utility>


bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

OctomapServer::OctomapServer( const ros::NodeHandle &nh_,std::string m_worldFrameId)
: m_nh(nh_),
  m_nh_private(nh_),
  m_octree(NULL),
  m_res(0.5),
  m_worldFrameId(m_worldFrameId),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_incrementalUpdate(false)
{
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  m_nh_private.param("resolution", m_res, m_res);
  m_gridmap.info.resolution = m_res;
  m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, true);


  // initialize octomap object & params

}

void OctomapServer::Initialize(Octree::Ptr octree)
{
  m_octree = octree;
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
}



void OctomapServer::publishProjected2DMap(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->getLeafCount();
  
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }


  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // call pre-traversal hook:
  handlePreNodeTraversal();

  // now, traverse all leafs in the tree:
  Eigen::Vector3f center;
  pcl::PointXYZINormal min_occupied,max_occupied;
  bool initial_check=true;  
  std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> voxelCenters;
  m_octree->getOccupiedVoxelCenters(voxelCenters);
  for (const auto& occupied_center : voxelCenters) {
      if(!initial_check)
      {
      min_occupied.x = std::min(occupied_center.x, min_occupied.x);
      min_occupied.y = std::min(occupied_center.y, min_occupied.y);
      min_occupied.z = std::min(occupied_center.z, min_occupied.z);
      max_occupied.x = std::min(occupied_center.x, max_occupied.x);
      max_occupied.y = std::min(occupied_center.y, max_occupied.y);
      max_occupied.z = std::min(occupied_center.z, max_occupied.z);
      }
      else {
      min_occupied = occupied_center;
      max_occupied = occupied_center;
      initial_check=false;
      }
      update2DMap(occupied_center, true);
  }
  // handlePostNodeTraversal(rostime,minPt, maxPt)
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("MinPt: %f %f %f / MaxPt: %f %f %f", min_occupied.x, min_occupied.y, min_occupied.z, max_occupied.x, max_occupied.y, max_occupied.z);
  m_gridmap.header.stamp = rostime;
  m_mapPub.publish(m_gridmap);
  ROS_INFO("Map publishing in OctomapServer took %f sec", total_elapsed);
}

// std::vector<int> cropRectangleFrom1D(const std::vector<int>& data, int width, int height, int x_start, int y_start, int x_end, int y_end) {
//     std::vector<int> croppedData;

//     return croppedData;
// }

// void OctomapServer::handlePostNodeTraversal(const ros::Time& rostime, pcl::PointXYZINormal minPt,pcl::PointXYZINormal maxPt){                                                        
//     // init projected 2D map:
//     m_gridmap.header.frame_id = m_worldFrameId;
//     m_gridmap.header.stamp = rostime;

//     unsigned width = static_cast<unsigned int>(std::abs(m_res*(maxPt.x - minPt.x)));
//     unsigned height = static_cast<unsigned int>(std::abs(m_res*(maxPt.y - minPt.y)));    
//     unsigned y_start = static_cast<unsigned int>(std::abs(m_res*(m_gridmap.info.origin.position.y-minPt.y)));
//     unsigned x_start = static_cast<unsigned int>(std::abs(m_res*(m_gridmap.info.origin.position.x-minPt.x)));
//     unsigned y_end = static_cast<unsigned int>(std::abs(m_res*(m_gridmap.info.origin.position.y-maxPt.y)));
//     unsigned x_end = static_cast<unsigned int>(std::abs(m_res*(m_gridmap.info.origin.position.x-maxPt.x)));
//     std::vector<int> occupancyGridData(m_gridmap.data.begin(), m_gridmap.data.end());
//     std::vector<int> temp(width * height);
//     for (int y = y_start; y < y_end; ++y) {
//         for (int x = x_start; x < x_end; ++x) {
//             int old_index = y * width + x;
//             int new_index = (y - y_start) * width + (x - x_start);
//             temp[new_index] = m_gridmap.data[old_index];
//         }
//     }
//     m_gridmap.info.width = static_cast<unsigned int>(std::abs(m_res*(maxPt.x - minPt.x)));
//     m_gridmap.info.height = static_cast<unsigned int>(std::abs(m_res*(maxPt.y - minPt.y)));
//     m_gridmap.info.resolution = m_res;
//     m_gridmap.info.origin.position.x = minPt.x;
//     m_gridmap.info.origin.position.y = minPt.y;
//     m_gridmap.data=std::move(temp);
// }


void OctomapServer::handlePreNodeTraversal(){                                                        
    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
  
    minPt = pcl::PointXYZINormal({static_cast<float>(minX), static_cast<float>(minY), static_cast<float>(minZ), 0.5f, 0.0f, 1.0f, 0.0f, 0.1f});
    maxPt = pcl::PointXYZINormal({static_cast<float>(maxX), static_cast<float>(maxY), static_cast<float>(maxZ), 0.5f, 0.0f, 1.0f, 0.0f, 0.1f});

    ROS_INFO("MinPt: %f %f %f / MaxPt: %f %f %f", minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z);

    m_gridmap.info.width = static_cast<unsigned int>(std::abs(m_res*(maxPt.x - minPt.x)));
    m_gridmap.info.height = static_cast<unsigned int>(std::abs(m_res*(maxPt.y - minPt.y)));
    m_gridmap.info.resolution = m_res;
    m_gridmap.info.origin.position.x = minPt.x;
    m_gridmap.info.origin.position.y = minPt.y;
    m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);
}

void OctomapServer::update2DMap(pcl::PointXYZINormal current_3dpoint, bool occupied){
  // update 2D map (occupied always overrides):
  int i =  static_cast<unsigned int>(std::abs(m_res*(current_3dpoint.x - minPt.x)));
  int j =  static_cast<unsigned int>(std::abs(m_res*(current_3dpoint.y - minPt.y)));
  unsigned idx = mapIdx(i, j);
  if (occupied)
  {
    m_gridmap.data[idx] = 100;
  }
  else
  {
      m_gridmap.data[idx] = 0;
  }
}