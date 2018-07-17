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
#ifndef _trajectory_planner_nodelet_H_
#define _trajectory_planner_nodelet_H_

/**
 * @file trajectory_planner_nodelet.h
 * @brief include file from trajectory_planner_nodelet
 * @author Joel Pereira
 * @version v0
 * @date 2012-05-10
 */

/**
 * @file trajectory_planner_nodelet.h
 * @brief Include file from trajectory_planner_nodelet
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <atlasmv_base/AtlasmvStatus.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <mtt/TargetListPC.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <trajectory_planner/c_manage_trajectory.h>
#include <trajectory_planner/c_trajectory.h>
#include <trajectory_planner/coordinates.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// #include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>

// Defines
#define _D_ 2.550
#define _NUM_TRAJ_ 25
#define _NUM_NODES_ 10

#define _VEHICLE_HEIGHT_TOP_ 1.460
#define _VEHICLE_HEIGHT_BOTTOM_ 0.150
#define _VEHICLE_LENGHT_FRONT_ 0.620 + _D_
#define _VEHICLE_LENGHT_BACK_ 0.425
#define _VEHICLE_WIDTH_ 1.775

#define _MAX_STEERING_ANGLE_ 41.5
#define _TRAJECTORY_ANGLE_ 3

// namepaces
using namespace visualization_msgs;

#ifdef _trajectory_planner_nodelet_CPP_
#define _EXTERN_
#else
#define _EXTERN_ extern
#endif

//   ___________________________________
//   |                                 |
//   |        PROTOTYPES               |
//   |_________________________________|
//   Defined in trajectory_planner_nodelet.cpp
void StatusMessageHandler(const atlasmv_base::AtlasmvStatus& msg);
int main(int argc, char** argv);
bool jump_node(double dist_init, int node, c_trajectoryPtr t);
void send_command_message(vector<double> speed_setted, int current_node, c_trajectoryPtr t);
void pcl_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
void line_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
void velocity_callback(double speed);
void velocity_update_callback(double speed);
double angle_to_speed(double angle);
double compute_last_dir(double angle);

// Global Vars
_EXTERN_ ros::NodeHandle* p_n;
_EXTERN_ tf::TransformListener* p_listener;
_EXTERN_ tf::StampedTransform transformw;
_EXTERN_ tf::StampedTransform transform_mtt;
_EXTERN_ ros::Publisher commandPublisher;
_EXTERN_ c_manage_trajectoryPtr manage_vt;
_EXTERN_ bool _simulation_;

#endif
