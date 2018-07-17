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
#ifndef _traj_executor_CPP_
#define _traj_executor_CPP_

/**
 * @file trajectory_executive.cpp
 * @brief publishes a command message to follow a trajectory
 * @author Joel Pereira
 * @version v0
 * @date 2012-05-25
 */

// Includes
#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <atlasmv_base/AtlasmvStatus.h>
#include <math.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <trajectory_planner/c_manage_trajectory.h>
#include <trajectory_planner/c_trajectory.h>
#include <trajectory_planner/traj_info.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

// Global vars
ros::Publisher commandPublisher;
trajectory_planner::traj_info info;
atlasmv_base::AtlasmvMotionCommand command;
ros::NodeHandle* p_n;
double dist_init;
atlasmv_base::AtlasmvStatus base_status;

/**
 * @brief Callback of published message
 * @details Comand message send to robotic vehicle
 * @param int
 * @return void
 */
void send_command_message(int current_node)
{
	if (current_node != -1 && static_cast<int>(info.alpha.size()) > current_node)
	{
		command.dir = info.alpha[current_node];
		command.speed = info.speed[current_node];
		command.lifetime = INFINITY;
		command.priority = 2;
		// std::cout<<"DIR -> "<<command.dir<<std::endl;
		// std::cout<<"SPEED -> "<<command.speed<<std::endl;
	}
	else
	{
		command.dir = 0.0;
		command.speed = 0.0;
		command.lifetime = INFINITY;
		command.priority = 2;
	}
	command.header.stamp = ros::Time::now();
	commandPublisher.publish(command);
	struct timeval tv;
	struct timezone tz;
	struct tm* tm;
	gettimeofday(&tv, &tz);
	tm = localtime(&tv.tv_sec);
	// printf("Tempooexec %d:%02d:%02d %d \n", tm->tm_hour, tm->tm_min, tm->tm_sec, tv.tv_usec);
}

/**
 * @brief Determines if the robot is already on the following node
 * @param double dist_init
 * @param int node
 * @param c_trajectoryPtr t
 * @return bool
 */
bool jump_node(double dist_init, int node)
{
	if (node != -1 && static_cast<int>(info.arc.size()) > node)
	{
		if (info.arc[node] > 0)
		{
			if (info.total_arc[node] <= (base_status.distance_traveled - dist_init))
			{
				return true;
			}
		}
		else
		{
			if (info.total_arc[node] >= (base_status.distance_traveled - dist_init))
			{
				return true;
			}
		}
	}
	return false;
}

/**
 * @brief Define message status
 * @param const atlasmv_base::AtlasmvStatus& msg
 * @return void
 */
void StatusMessageHandler(const atlasmv_base::AtlasmvStatus& msg)
{
	base_status = msg;
}

/**
 * @brief
 * @param const trajectory_planner::traj_info& msg
 * @return void
 */
void Info_handler(const trajectory_planner::traj_info& msg)
{
	info = msg;
	dist_init = base_status.distance_traveled;
	// 	cout<<"DISTANCE TRAVELED "<< dist_init<<endl;
}

/**
 * @brief Main code of the nodelet
 * @details Publishes the trajectories message and the command message
 * @param int argc
 * @param char **argv
 * @return int
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_executor_nodelet");
	ros::NodeHandle n("~");
	p_n = &n;

	// Define the publishers and subscribers
	ros::Subscriber subscribe_traj_info = n.subscribe("/trajectory_information", 1, Info_handler);
	ros::Subscriber subscribeStatusMessages = n.subscribe("/atlasmv/base/status", 1, StatusMessageHandler);
	ros::Rate loop_rate(10);

	// Follow chosen trajectory
	commandPublisher = n.advertise<atlasmv_base::AtlasmvMotionCommand>("/atlasmv/base/motion", 1);
	int node = 0;

	while (ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();

		if (info.arc.size() == 0)
			continue;

		//   ___________________________________
		//   |                                 |
		//   |        Trajectory execution     |
		//   |_________________________________|

		bool jump_to_next_node = jump_node(dist_init, node);
		if (jump_to_next_node)
		{
			if (node == static_cast<int>((info.arc.size() - 1)))
			{
				node = -1;
			}
			else
			{
				node++;
			}
		}
		// 		cout<<"NODE "<<node<<" distance "<<base_status.distance_traveled<<endl;

		send_command_message(node);
	}
}
#endif
