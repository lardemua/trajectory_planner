/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/**
 * @file mtt_target_generator.cpp
 * @brief Interactive marker obstacle generator
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file mtt_target_generator.cpp
 * @brief Interactive marker obstacle generator
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <math.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <numeric>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <mtt/TargetListPC.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "std_msgs/String.h"

// namepaces
using namespace visualization_msgs;

// Global Vars
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::NodeHandle *p_n;
ros::Publisher coor_pub;
visualization_msgs::Marker marker;
ros::Publisher line_pub;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
	// mouse_point_ss << " at " << feedback->mouse_point.x
	//<< ", " << feedback->mouse_point.y
	//<< ", " << feedback->mouse_point.z
	//<< " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

	  mtt::TargetListPC message;
	  message.header.stamp = ros::Time::now();
	  message.header.frame_id = "/vehicle_odometry";

	  // set the mtt msg id field
	  message.id.push_back(0);

	  // set the mtt msg position field
	  pcl::PointCloud<pcl::PointXYZ> pc_in;
	  pcl::PointXYZ pt;
	  pt.x = feedback->pose.position.x;
	  pt.y = feedback->pose.position.y;
	  pt.z = 0;
	  pc_in.points.push_back(pt);
	  pcl::toROSMsg(pc_in, message.position);

	  // set the mtt msg velocity field
	  pcl::PointCloud<pcl::PointXYZ> pc_vel;
	  pt.x = 0;
	  pt.y = 0;
	  pt.z = 0;
	  pc_vel.points.push_back(pt);
	  pcl::toROSMsg(pc_vel, message.velocity);

	  // set the mtt msg obstacle_lines field
	  pcl::PointCloud<pcl::PointXYZ> pc_obs;
	  pt.x = feedback->pose.position.x + 0;
	  pt.y = feedback->pose.position.y - 1;
	  pt.z = 0;
	  pc_obs.points.push_back(pt);

	  pt.x = feedback->pose.position.x + 0;
	  pt.y = feedback->pose.position.y + 1;
	  pt.z = 0;
	  pc_obs.points.push_back(pt);

	  pt.x = feedback->pose.position.x + 1;
	  pt.y = feedback->pose.position.y + 1;
	  pt.z = 0;
	  pc_obs.points.push_back(pt);

	  sensor_msgs::PointCloud2 pc_msg;
	  pcl::toROSMsg(pc_obs, pc_msg);

	  message.obstacle_lines.push_back(pc_msg);

	  // publish the mtt msg
	  coor_pub.publish(message);

	  // change the position of the line vis marker
	  geometry_msgs::Point p;
	  marker.header.frame_id = "/vehicle_odometry";
	  marker.header.stamp = ros::Time::now();
	  marker.ns = "lines";
	  marker.id = 0;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.type = visualization_msgs::Marker::LINE_LIST;
	  marker.scale.x = 0.015;
	  marker.color.r = 0.7;
	  marker.color.g = 0.1;
	  marker.color.b = 0.1;
	  marker.color.a = 1.0;
	  marker.pose.position.x = feedback->pose.position.x;
	  marker.pose.position.y = feedback->pose.position.y;
	  marker.pose.position.z = 0;

	  p.x = 0;
	  p.y = -1;
	  p.z = 0;
	  marker.points.push_back(p);
	  p.x = 0;
	  p.y = 1;
	  p.z = 0;
	  marker.points.push_back(p);
	  marker.points.push_back(p);
	  p.x = 1;
	  p.y = 1;
	  p.z = 0;
	  marker.points.push_back(p);

	  line_pub.publish(marker);

	  break;
  }

  server->applyChanges();
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale / 7;
  marker.scale.y = msg.scale / 7;
  marker.scale.z = msg.scale / 7;
  marker.color.r = 0.7;
  marker.color.g = 0.1;
  marker.color.b = 0.1;
  marker.color.a = 0.8;
  return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void make6DofMarker(bool fixed)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/vehicle_odometry";
  int_marker.pose.position.x = 5;
  int_marker.pose.position.y = 4;
  int_marker.pose.position.z = 0;
  int_marker.scale = 0.4;
  int_marker.name = "control_obstacle";
  int_marker.description = "Control the final \nposition of the obstacle";

  // insert a box
  makeBoxControl(int_marker);
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtt_target_generator");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  p_n = &n;

  coor_pub = n.advertise<mtt::TargetListPC>("/mtt_targets", 1000);

  line_pub = n.advertise<visualization_msgs::Marker>("/line_markers", 1);

  server.reset(new interactive_markers::InteractiveMarkerServer("mtt_target_generator/im", "", false));
  ros::Duration(0.1).sleep();
  make6DofMarker(true);
  server->applyChanges();

  while (ros::ok())
  {
	loop_rate.sleep();
	ros::spinOnce();
  }

  server.reset();
}
