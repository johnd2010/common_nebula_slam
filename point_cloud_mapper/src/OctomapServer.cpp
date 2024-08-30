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

void OctomapServer::handlePreNodeTraversal(){                                                        
    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
    minPt = pcl::PointXYZINormal({static_cast<float>(minX), static_cast<float>(minY), static_cast<float>(minZ), 0.5f, 0.0f, 1.0f, 0.0f, 0.1f});
    maxPt = pcl::PointXYZINormal({static_cast<float>(maxX), static_cast<float>(maxY), static_cast<float>(maxZ), 0.5f, 0.0f, 1.0f, 0.0f, 0.1f});
    m_gridmap.info.width = static_cast<unsigned int>(std::abs(m_res*(maxPt.x - minPt.x)));
    m_gridmap.info.height = static_cast<unsigned int>(std::abs(m_res*(maxPt.y - minPt.y)));
    m_gridmap.info.resolution = m_res;
    m_gridmap.info.origin.position.x = minPt.x;
    m_gridmap.info.origin.position.y = minPt.y;
    m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);
}

void OctomapServer::publishProjected2DMap(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->getLeafCount();
  
  
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }
  // call pre-traversal hook:
  // handlePreNodeTraversal();

  // now, traverse all occupied voxels in the tree:
  getOccupiedLimits();

  handlePostNodeTraversal();
  std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> voxelCenters;
  m_octree->getOccupiedVoxelCenters(voxelCenters);  
  
  for (const auto& occupied_center : voxelCenters)
    update2DMap(occupied_center, true);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Min Occupied: %f %f %f / Max Occupied: %f %f %f", min_occupied.x, min_occupied.y, min_occupied.z, max_occupied.x, max_occupied.y, max_occupied.z);
  ROS_INFO("Width: %u  Height: %u", m_gridmap.info.width, m_gridmap.info.height);
  m_gridmap.header.frame_id = m_worldFrameId;
  m_gridmap.header.stamp = rostime;
  m_mapPub.publish(m_gridmap);
  ROS_INFO("Map publishing in OctomapServer took %f sec", total_elapsed);
}

void OctomapServer::handlePostNodeTraversal(){                                                        
    m_gridmap.info.width = static_cast<unsigned int>(std::abs((max_occupied.x - min_occupied.x))/m_res);
    m_gridmap.info.height = static_cast<unsigned int>(std::abs((max_occupied.y - min_occupied.y))/m_res);
    m_gridmap.info.resolution = m_res;
    m_gridmap.info.origin.position.x = min_occupied.x;
    m_gridmap.info.origin.position.y = min_occupied.y;
    m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);
}

void OctomapServer::update2DMap(pcl::PointXYZINormal current_3dpoint, bool occupied){
  // update 2D map (occupied always overrides):
  int i =  static_cast<unsigned int>(std::abs((current_3dpoint.x - m_gridmap.info.origin.position.x)/m_res));
  int j =  static_cast<unsigned int>(std::abs((current_3dpoint.y - m_gridmap.info.origin.position.y)/m_res));
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

void OctomapServer::getOccupiedLimits()
{
  std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> voxelCenters;
  m_octree->getOccupiedVoxelCenters(voxelCenters);  
  for (const auto& occupied_center : voxelCenters) {
    if(!initial_check)
    {
    min_occupied.x = std::min(occupied_center.x, min_occupied.x);
    min_occupied.y = std::min(occupied_center.y, min_occupied.y);
    min_occupied.z = std::min(occupied_center.z, min_occupied.z);
    max_occupied.x = std::max(occupied_center.x, max_occupied.x);
    max_occupied.y = std::max(occupied_center.y, max_occupied.y);
    max_occupied.z = std::max(occupied_center.z, max_occupied.z);
    }
    else {
    min_occupied = occupied_center;
    max_occupied = occupied_center;
    initial_check=false;
    }
  }
}