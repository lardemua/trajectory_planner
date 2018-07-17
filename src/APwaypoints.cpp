/**************************************************************************************************
   Software License Agreement (BSD License)
   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.
   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
   \@file  APwaypoints.cpp
   \@brief Integration of global navigation way points
   \@author Ricardo Silva
    @version v0
   \@date 2018-06-06
 */

#include <trajectory_planner/trajectory_planner_nodelet.h>

namespace waypoints_analise
{
/**
 @brief Class to handle with the way points
*/
class waypointsAnalise
{
public:
  waypointsAnalise(std::string topicName, std::string frame_id);

  void waypointsDataTreatment(const std_msgs::Float64MultiArray::ConstPtr& waypoints);

  geometry_msgs::PointStamped computeAP(const std_msgs::Float64MultiArray& polygon_in);

private:
  ros::NodeHandle n;

  ros::Subscriber waypointsSub;
  ros::Publisher APPub;
  ros::Publisher WayPreviousPub;
  ros::Publisher WayNextPub;
  ros::Publisher APtargetPub;

  std::string topicName;
  std::string frameId;

  trajectory_planner::coordinates AP;
  geometry_msgs::PointStamped WayPrevious;
  geometry_msgs::PointStamped WayNext;
  geometry_msgs::PointStamped APtarget;
};

/**
   @brief waypointsAnalise class constructor
   @param Way points topic
   @param Reference frame
   @return void
 */
waypointsAnalise::waypointsAnalise(std::string topicName, std::string frame_id)
{
  this->topicName = topicName;
  this->frameId = frame_id;

  waypointsSub = n.subscribe(topicName, 1, &waypointsAnalise::waypointsDataTreatment, this);
  APPub = n.advertise<trajectory_planner::coordinates>("msg_coordinates", 1);
  WayPreviousPub = n.advertise<geometry_msgs::PointStamped>("previous_way_point", 1);
  WayNextPub = n.advertise<geometry_msgs::PointStamped>("next_way_point", 1);
  APtargetPub = n.advertise<geometry_msgs::PointStamped>("ap_target", 1);
  ROS_INFO("Topic %s subscribed!", topicName.c_str());
}

/**
   @brief Function to deal with previous and next way points and publish the algorithm's target
   @param Way points array
   @return void
 */
void waypointsAnalise::waypointsDataTreatment(const std_msgs::Float64MultiArray::ConstPtr& waypoints)
{

    geometry_msgs::PointStamped previous;
    geometry_msgs::PointStamped next;
    previous.header.frame_id = this->frameId;
    next.header.frame_id = this->frameId;

    previous.point.x = waypoints->data[0];
    previous.point.y = waypoints->data[2];
    previous.point.z = 0;
    next.point.x = waypoints->data[1];
    next.point.y = waypoints->data[3];
    next.point.z = 0;

    /*---Publisher for the WayPoints--*/
    WayPreviousPub.publish(previous);
    WayNextPub.publish(next);


    geometry_msgs::PointStamped apoint;
    apoint = computeAP(*waypoints);
    apoint.header.frame_id = this->frameId;

    /*---Publisher for the APPoint--*/
    APtargetPub.publish(apoint);

    AP.x = apoint.point.x;
    AP.y = apoint.point.y;
    AP.theta = 0;
    APPub.publish(AP);

}

/**
   @brief Function to calculate target point in a circle of r radius
   @param Way points array
   @return Target point
 */
geometry_msgs::PointStamped waypointsAnalise::computeAP(const std_msgs::Float64MultiArray& waypoints_in)
{
  double r = 12;
  geometry_msgs::PointStamped point;
  double m = (waypoints_in.data[3] - waypoints_in.data[2]) / (waypoints_in.data[1] - waypoints_in.data[0]);
  double b = waypoints_in.data[3] - m * waypoints_in.data[1];

  double x = (-2 * m * b + pow((pow((2 * m * b), 2) - 4 * ((pow(m, 2) + 1) * (pow(b, 2) - pow(r, 2)))), 0.5)) /
             (2 * (pow(m, 2) + 1));

  point.point.x = x;
  point.point.y = (m * x + b);
  point.point.z = 0;

  return point;
}
}

/**
   @brief Main function to compute target point
   @param argc
   @param argv
   @return int
 */
int main(int argc, char** argv)
{
  /*-- Node initialization --*/
  ros::init(argc, argv, "APwaypoints");

  ros::NodeHandle n;  // node "local_path_planning" access point

  waypoints_analise::waypointsAnalise waypointsAnalise("/waypoints_previous_next", "/base_link_imu");

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
