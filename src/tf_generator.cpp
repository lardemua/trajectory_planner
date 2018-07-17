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
 * @file tf_generator.cpp
 * @brief Transform frame generator
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file tf_generator.cpp
 * @brief Transform frame generator
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_planner/trajectory_planner_nodelet.h>
tf::TransformBroadcaster* p_broadcaster;

/**
 * @brief Generates a frame higher than planning_world, to publish the trajectories origin
 * @param int
 * @param char**
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_generator_node");
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;
  p_broadcaster = &broadcaster;
  ros::Rate r(50);

  double theta = 0;
  int inc = 0;
  while (n.ok())
  {
    theta = inc * (2 * M_PI) / 1000;

    tf::Transform transform(tf::Matrix3x3(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1),
                            tf::Vector3(-(0.58 + _D_) * cos(theta), -(0.58 + _D_) * sin(theta), 0.0));

    p_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/planning_world", "/vehicle_odometry"));
    r.sleep();
    ros::spinOnce();
  }
}
