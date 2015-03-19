// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "graph_ros_publisher.h"

#include <sstream>
#include <map>

GraphRosPublisher::GraphRosPublisher(GraphSLAM& graph, string fixedFrame,
                                     int nRobots)
    : _graph(graph), _fixedFrame(fixedFrame) {

  for (int i = 0; i < nRobots; ++i){
    std::ostringstream sstream;
    sstream << "poses_robot_" << i;
    _pubPosesRobots.push_back(_nh.advertise<geometry_msgs::PoseArray>(sstream.str(), 
                                                                      1, true));
  }
  _pubMapSelf = _nh.advertise<sensor_msgs::PointCloud>("map_self", 1, true);
  _pubMapOthers = _nh.advertise<sensor_msgs::PointCloud>("map_others", 1, true);
}

void GraphRosPublisher::publishGraph(){
  
  // First build a map from the unordered_map containing the vertices to sort
  // them
  std::map<int, HyperGraph::Vertex*> vertices;
  for (HyperGraph::VertexIDMap::const_iterator it=_graph.graph()->vertices().begin(); 
       it!=_graph.graph()->vertices().end(); ++it){
    vertices.insert(*it);
  }

  sensor_msgs::PointCloud map_self;
  sensor_msgs::PointCloud map_others;

  const int base_id = _graph.baseId();
  const int my_robot_id = _graph.idRobot();
  std::map<int, HyperGraph::Vertex*>::const_iterator it, start, end;
  end = vertices.begin(); // Make the first iteration work
  geometry_msgs::PoseArray poses;

  for (int robot_id = 0; static_cast<size_t>(robot_id) < _pubPosesRobots.size(); 
       ++robot_id){
    poses.poses.clear();

    // Find vertices from robot_id
    start = end;
    end   = vertices.upper_bound((robot_id+1) * base_id - 1);
    
    for (it = start; it != end; ++it){
      VertexSE2* v = static_cast<VertexSE2*>(it->second);
      poses.poses.resize(poses.poses.size() + 1);
      geometry_msgs::Pose& new_pose = poses.poses.back();
      new_pose.position.x = v->estimate().translation().x();
      new_pose.position.y = v->estimate().translation().y();
      new_pose.position.z = 0;
      new_pose.orientation = tf::createQuaternionMsgFromYaw(v->estimate().rotation().angle());

      RobotLaser *laser = dynamic_cast<RobotLaser*>(v->userData());
      if (laser){
        RawLaser::Point2DVector vscan = laser->cartesian();
        SE2 trl = laser->laserParams().laserPose;
        SE2 transf = v->estimate() * trl;
        RawLaser::Point2DVector wscan;
        ScanMatcher::applyTransfToScan(transf, vscan, wscan);
            
        size_t s= 0;
        while (s<wscan.size()){
          sensor_msgs::PointCloud& point_cloud = (robot_id == my_robot_id
                                                  ? map_self : map_others);
          point_cloud.points.resize(point_cloud.points.size() + 1);
          geometry_msgs::Point32& new_point = point_cloud.points.back();
          new_point.x = wscan[s].x();
          new_point.y = wscan[s].y();
          
          s = s+10;
        }
      }
    }

    if (!poses.poses.empty()){
      poses.header.frame_id = _fixedFrame;
      _pubPosesRobots[robot_id].publish(poses);
    }
  }
  
  map_self.header.frame_id = map_others.header.frame_id = _fixedFrame;
  _pubMapSelf.publish(map_self);
  _pubMapOthers.publish(map_others);
}
