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
 * @file APgenerator.cpp
 * @brief Interactive marker atractor point generator
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file APgenerator.cpp
 * @brief Interactive marker atractor point generator
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <math.h>
#include <stdio.h>
#include <trajectory_planner/coordinates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <numeric>
#include <vector>
#include "std_msgs/String.h"

// namepaces
using namespace visualization_msgs;

// Global Vars
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::NodeHandle *p_n;
ros::Publisher coor_pub;
trajectory_planner::coordinates message;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // ROS_INFO("Here");
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
      message.x = feedback->pose.position.x;
      message.y = feedback->pose.position.y;
      message.theta = 2.0 * asin(feedback->pose.orientation.z);

      message.header.stamp = ros::Time::now();
      message.header.frame_id = "/planning_world";

      coor_pub.publish(message);

      break;
  }

  server->applyChanges();
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale / 2;
  marker.scale.y = msg.scale / 6;
  marker.scale.z = msg.scale / 6;
  marker.color.r = 0;
  marker.color.g = 0.7;
  marker.color.b = 0;
  marker.color.a = 0.6;
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

InteractiveMarker make6DofMarker(bool fixed)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/planning_world";
  int_marker.pose.position.x = 6;
  int_marker.pose.position.y = 0;
  int_marker.pose.position.z = 0;
  int_marker.scale = 0.6;

  int_marker.name = "Control target";
  int_marker.description = "Control the final \nposition of the robot";

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

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.name = "rotate_z";
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  return int_marker;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "APgenerator");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  p_n = &n;

  coor_pub = n.advertise<trajectory_planner::coordinates>("/msg_coordinates", 1000);

  server.reset(new interactive_markers::InteractiveMarkerServer("APgenerator/im", "", false));
  ros::Duration(0.1).sleep();
  InteractiveMarker marker = make6DofMarker(true);

  server->applyChanges();

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  server.reset();
}
