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
#ifndef _trajectory_planner_nodelet_aux_CPP_
#define _trajectory_planner_nodelet_aux_CPP_

/**
 * @file trajectory_executive.cpp
 * @brief publishes a command message to follow a trajectory
 * @author Joel Pereira
 * @version v0
 * @date 2012-05-25
 */

#include <trajectory_planner/trajectory_planner_nodelet.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
                   << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      manage_vt->set_attractor_point(feedback->pose.position.x, feedback->pose.position.y,
                                     2.0 * asin(feedback->pose.orientation.z));

      break;
  }

  server->applyChanges();
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;
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

void make6DofMarker(bool fixed)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/vehicle_odometry";
  int_marker.pose.position.x = 5;
  int_marker.pose.position.y = 0;
  int_marker.pose.position.z = 0;
  int_marker.scale = 0.4;

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
}
#endif
