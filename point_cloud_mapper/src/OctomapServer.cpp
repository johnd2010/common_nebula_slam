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


#include "opencv2/core/types.hpp"
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
  
}

void OctomapServer::Initialize(Octree::Ptr octree)
{
  m_nh_private.param("map/occupancy_map_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("map/occupancy_map_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  m_nh_private.param("map/occupancy_map_resolution", m_res, m_res);
  m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, true);
  m_gridmap.info.resolution = m_res;
  ROS_INFO("Occupancy Map with Resolution %f and Z (MinZ,MaxZ) : (%f,%f)",m_res,m_occupancyMinZ,m_occupancyMaxZ);
  m_octree = octree;
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

}


void OctomapServer::publishProjected2DMap(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->getLeafCount();
  oldMapInfo = m_gridmap.info;
  
  
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }
  // call pre-traversal hook:

  // now, traverse all occupied voxels in the tree:
  
  m_octree->getOccupiedVoxelCenters(voxelCenters);  
  getOccupiedLimits();

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  m_gridmap.header.frame_id = m_worldFrameId;
  m_gridmap.header.stamp = rostime;
  m_mapPub.publish(m_gridmap);
  ROS_INFO("Map publishing in OctomapServer took %f sec", total_elapsed);
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

void OctomapServer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);
  }

}

void OctomapServer::getOccupiedLimits()
{
  initial_check = true;
  filteredVoxelCloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
  convexCloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
  downsampledCloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
  //downsample pointcloud
  sor.setInputCloud(m_octree->getInputCloud());
  sor.setLeafSize(m_res,m_res,m_res);
  sor.filter(*downsampledCloud);
  //crop pointcloud
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_occupancyMinZ,m_occupancyMaxZ);
  pass.setInputCloud(downsampledCloud);
  pass.filter(*filteredVoxelCloud);
  //get concave points
  chull.setInputCloud(filteredVoxelCloud);
  chull.setAlpha (0.1);
  chull.setDimension(2);
  chull.reconstruct(*convexCloud);
  initializeOccupancyMap();
}


void OctomapServer::initializeOccupancyMap(){     
  
    std::vector<pcl::PointXYZINormal> polygonVertices;
    cv::Point occ_point,occ_centroid;
    Eigen::Vector4f centroid;
    initial_check=true;
    pcl::compute3DCentroid(*convexCloud,centroid);
    for (const auto& concave_points : *convexCloud) {
      insertPointClockwise(polygonVertices, centroid, concave_points);
      if(!initial_check){
        min_occupied.x = std::min(concave_points.x, min_occupied.x);
        min_occupied.y = std::min(concave_points.y, min_occupied.y);
        min_occupied.z = std::min(concave_points.z, min_occupied.z);
        max_occupied.x = std::max(concave_points.x, max_occupied.x);
        max_occupied.y = std::max(concave_points.y, max_occupied.y);
        max_occupied.z = std::max(concave_points.z, max_occupied.z);
      }
      else {
        min_occupied = concave_points;
        max_occupied = concave_points;
        initial_check=false;
      }
    }
    m_gridmap.info.width = static_cast<unsigned int>(std::abs((max_occupied.x - min_occupied.x))/m_res);
    m_gridmap.info.height = static_cast<unsigned int>(std::abs((max_occupied.y - min_occupied.y))/m_res);
    m_gridmap.info.resolution = m_res;
    m_gridmap.info.origin.position.x = min_occupied.x;
    m_gridmap.info.origin.position.y = min_occupied.y;
    cv::Mat binary_occupancy_map(m_gridmap.info.height,m_gridmap.info.width,CV_8SC1, cv::Scalar(-1));

    for (const auto& occupied_center : *filteredVoxelCloud){      
      uchar* ptr = binary_occupancy_map.ptr<uchar>(static_cast<unsigned int>(std::abs((occupied_center.y - min_occupied.y))/m_res));
      ptr[static_cast<unsigned int>(std::abs((occupied_center.x - min_occupied.x))/m_res)] = 100;
    }
    std::vector<cv::Point> concave_occupancy;
    std::vector<std::vector<cv::Point>> polygons;
    for (const auto& point : polygonVertices) 
    {
      concave_occupancy.push_back(cv::Point((std::abs((point.x - min_occupied.x))/m_res),(std::abs((point.y - min_occupied.y))/m_res)));
    }
    polygons.push_back(concave_occupancy);

    ROS_INFO("before fillpoly");
    cv::fillPoly(binary_occupancy_map, polygons, cv::Scalar(0));
    std::vector<int8_t> occupancyGridData(binary_occupancy_map.begin<uchar>(), binary_occupancy_map.end<uchar>());
    m_gridmap.data = occupancyGridData;
}
