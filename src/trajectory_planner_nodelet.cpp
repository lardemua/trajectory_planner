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
#ifndef _trajectory_planner_nodelet_CPP_
#define _trajectory_planner_nodelet_CPP_

/**
 * @file trajectory_planner_nodelet.cpp
 * @brief Uses the c-trajectory class, to publish trajectories and send the message to follow one of them
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file trajectory_planner_nodelet.cpp
 * @brief Uses the c-trajectory class, to publish trajectories and send the message to follow one of them
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <trajectory_planner/trajectory_planner_nodelet.h>

bool plan_trajectory = false;
bool have_plan = false;
bool have_trajectory = false;
vector<double> last_dir{ 0, 0, 0, 0, 0 };
geometry_msgs::PoseStamped pose_in;
geometry_msgs::PoseStamped pose_transformed;

std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_v;
//-------------------------------------------------------------------------------------------//
std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_v1;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptr(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_v2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptrl(new pcl::PointCloud<pcl::PointXYZ>);

/**
 * @brief Set attractor point coordinates
 * @param trajectory_planner::coordinates msg
 * @return void
 */
void set_coordinates(trajectory_planner::coordinates msg)
{
	// 	cout<<"stat Message received!!!"<<endl;
	// Change parameters
	pose_in.pose.position.x = msg.x;
	pose_in.pose.position.y = msg.y;
	pose_in.pose.orientation.w = cos(msg.theta / 2.0);
	pose_in.pose.orientation.x = 0.0;
	pose_in.pose.orientation.y = 0.0;
	pose_in.pose.orientation.z = sin(msg.theta / 2.0);

	pose_in.header.frame_id = "/planning_world";

	manage_vt->set_attractor_point(msg.x, msg.y, msg.theta);
	plan_trajectory = true;
}

/**
 * @brief Set mtt points
 * @param mtt::TargetListPC msg
 * @return void
 */
void mtt_callback(mtt::TargetListPC msg)
{
	// ROS_INFO("received mtt num obs=%ld num lines first obs =%d frame_id=%s", msg.id.size(),
	// msg.obstacle_lines[0].width,
	//  msg.header.frame_id.c_str());

	// copy to global variable
	pc_v.erase(pc_v.begin(), pc_v.end());
	for (size_t i = 0; i < msg.id.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ> tmp;
		pcl::fromROSMsg(msg.obstacle_lines[i], tmp);
		tmp.header.frame_id = msg.header.frame_id;
		// 		tmp.header.stamp=msg.header.stamp;

		// ROS_INFO("Obstacle %ld has %ld lines", i, tmp.points.size());
		pc_v.push_back(tmp);
	}
}

/**
 * @brief Set polygon points
 * @param mtt::TargetListPC msg
 * @return void
 */
void pcl_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, *pc_v_ptr);

	pc_v1.erase(pc_v1.begin(), pc_v1.end());

	pc_v1.push_back(*pc_v_ptr);
	// plan_trajectory = true;
}

/**
 * @brief Set line points
 * @param mtt::TargetListPC msg
 * @return void
 */
void line_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, *pc_v_ptrl);

	pc_v2.erase(pc_v2.begin(), pc_v2.end());

	pc_v2.push_back(*pc_v_ptrl);
	// plan_trajectory = true;
}

/**
 * @brief Function that generates a few number of trajectories from the defines _NUM_TRAJ_, _NUM_NODES_ and
 * _TRAJECTORY_ANGLE_
 * @param vehicle speed
 * @return void
 */
void velocity_callback(double speed)
{
	double max_dist = pow(speed * 3.6, 2) / 100;
	if (max_dist > 20)
	{
		max_dist = 20;
	}
	if (max_dist < 4.5)
	{
		max_dist = 4.5;
	}

	double i = 0.00000001;
	while (i < _MAX_STEERING_ANGLE_)
	{
		if (i == 0.00000001)
		{
			vector<double> v_a;
			vector<double> v_arc;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a.push_back(M_PI / 180. * i);
				v_arc.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->create_new_trajectory(v_a, v_arc, v_a);
		}
		else
		{
			vector<double> v_a1;
			vector<double> v_arc1;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a1.push_back(M_PI / 180. * i);
				v_arc1.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->create_new_trajectory(v_a1, v_arc1, v_a1);

			vector<double> v_a2;
			vector<double> v_arc2;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a2.push_back(M_PI / 180. * (-i));
				v_arc2.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->create_new_trajectory(v_a2, v_arc2, v_a2);
		}
		i = i + _TRAJECTORY_ANGLE_;
	}
	have_trajectory = true;
}

/**
 * @brief Function that updates trajectories during the planning
 * @param vehicle speed
 * @return void
 */
void velocity_update_callback(double speed)
{
	double max_dist = pow(speed * 3.6, 2) / 100;
	if (max_dist > 20)
	{
		max_dist = 20;
	}
	if (max_dist < 4.5)
	{
		max_dist = 4.5;
	}

	double i = 0.00000001;
	int num_traj = 0;
	while (i < _MAX_STEERING_ANGLE_)
	{
		if (i == 0.00000001)
		{
			vector<double> v_a;
			vector<double> v_arc;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a.push_back(M_PI / 180. * i);
				v_arc.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->update_trajectory(v_a, v_arc, v_a, num_traj);
			num_traj = num_traj + 1;
		}
		else
		{
			vector<double> v_a1;
			vector<double> v_arc1;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a1.push_back(M_PI / 180. * i);
				v_arc1.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->update_trajectory(v_a1, v_arc1, v_a1, num_traj);
			num_traj = num_traj + 1;

			vector<double> v_a2;
			vector<double> v_arc2;
			for (int j = 0; j < _NUM_NODES_; ++j)
			{
				v_a2.push_back(M_PI / 180. * (-i));
				v_arc2.push_back(max_dist / _NUM_NODES_);
			}
			manage_vt->update_trajectory(v_a2, v_arc2, v_a2, num_traj);
			num_traj = num_traj + 1;
		}
		i = i + _TRAJECTORY_ANGLE_;
	}
	have_trajectory = true;
}

/**
 * @brief Sets the speed in function of chosen trajectory angle
 * @param chosen trajectory angle
 * @return speed
 */
double angle_to_speed(double angle)
{
	double m = (_SPEED_SAFFETY_ - _SPEED_REQUIRED_) / (_MAX_STEERING_ANGLE_ * M_PI / 180);
	return (m * abs(angle) + _SPEED_REQUIRED_);
	// plan_trajectory = true;
}

/**
 * @brief Last five chosen directions filter
 * @param last chosen direction
 * @return mean
 */
double compute_last_dir(double angle)
{
	int n = last_dir.size();
	vector<double> last_dir_cp = last_dir;
	last_dir.clear();

	for (size_t i = 1; i < n; i++)
	{
		last_dir.push_back(last_dir_cp.at(i));
	}
	last_dir.push_back(angle);

	double mean = accumulate(last_dir.begin(), last_dir.end(), 0.0) / n;
	return mean;
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
	ros::init(argc, argv, "trajectory_planner_nodelet");
	ros::NodeHandle n("~");
	p_n = &n;

	if (n.getParam("/sim", _simulation_))
	{
		if (_simulation_)
		{
			ROS_INFO("Using Simulation!");
		}
		else
		{
			ROS_INFO("Not using Simulation!");
		}
	}
	else
	{
		ROS_WARN("Param 'sim' not found!");
	}

	// Define the publishers and subscribers
	tf::TransformBroadcaster mw_broadcaster;
	tf::TransformBroadcaster mtt_broadcaster;
	tf::TransformListener listener;
	p_listener = &listener;
	ros::Publisher array_pub = n.advertise<visualization_msgs::MarkerArray>("/array_of_markers", 1);
	ros::Subscriber sub = n.subscribe("/msg_coordinates", 1, set_coordinates);
	ros::Subscriber mtt_sub = n.subscribe("/mtt_targets", 1, mtt_callback);
	//----------------------------------------------------------------------------------------------------//
	// Polygon stuff
	ros::Subscriber pcl_sub = n.subscribe("/reduced_pcl", 1000, pcl_callback);
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/steer_drive_controller/cmd_vel", 1);
	ros::Publisher mindist_pub = n.advertise<std_msgs::Float64MultiArray>("/min_dist_to_obstacles", 1);

	// road lines
	ros::Subscriber line_sub;
	if (_simulation_)
	{
		line_sub = n.subscribe("/line_pcl", 1000, line_callback);
	}

	ros::Rate loop_rate(10);

	//   ___________________________________
	//   |                                 |
	//   |        Define the trajectories  |
	//   |_________________________________|
	// Declare the traj manager class
	manage_vt = (c_manage_trajectoryPtr) new c_manage_trajectory();

	// initialize attractor point
	t_desired_coordinates AP;
	AP.x = -1.0;
	AP.y = 0.0;
	AP.theta = -M_PI / 8;
	manage_vt->set_attractor_point(AP.x, AP.y, AP.theta);

	// initialize vehicle description
	manage_vt->set_vehicle_description(_VEHICLE_WIDTH_, _VEHICLE_LENGHT_BACK_, _VEHICLE_LENGHT_FRONT_,
																		 _VEHICLE_HEIGHT_TOP_, _VEHICLE_HEIGHT_BOTTOM_);

	// initialize inter axis distance
	manage_vt->set_inter_axis_distance(_D_);

	// initialize trajectories
	velocity_callback(_SPEED_SAFFETY_);

	//  trajectory information
	commandPublisher = n.advertise<trajectory_planner::traj_info>("/trajectory_information", 1000);

	double speed = _SPEED_SAFFETY_;
	while (ros::ok())
	{
		if (plan_trajectory == true)
		{
			// aqui recebi um comando para defenir uma trajectoria
			have_trajectory = false;
			velocity_update_callback(speed);

			//-------------------------------------------------------------------------//
			// plan_trajectory = false;
			// ROS_INFO("Going to plan a trajectory");
			// _________________________________
			//|                                 |
			//|    Set the parking view frame   |
			//|_________________________________|
			bool have_transform = true;
			// Set the frame where to draw the trajectories
			try
			{
				p_listener->lookupTransform("/planning_world", "/vehicle_odometry", ros::Time(0), transformw);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s", ex.what());
				have_transform = false;
			}

			if (have_transform & have_trajectory)
			{
				ros::Time time = ros::Time::now();
				mw_broadcaster.sendTransform(tf::StampedTransform(transformw, time, "/planning_world", "/vehicle_odometry"));
				ros::spinOnce();
				mw_broadcaster.sendTransform(tf::StampedTransform(transformw, time + ros::Duration(5), "/planning_world", "/vehicle_"
																																																				 "odometry"));
				ros::spinOnce();
				// 				cout<<"stat Publishing transform"<<endl;

				ros::Duration(0.1).sleep();

				//   ___________________________________
				//   |                                 |
				//   |        Trajectory evaluation    |
				//   |_________________________________|

				// Transform attractor point to /vehicle_odometry
				tf::Transformer tt;
				pose_in.header.stamp = time + ros::Duration(0.1);
				p_listener->transformPose("/vehicle_odometry", pose_in, pose_transformed);

				// ROS_INFO("pose_in frame_id=%s pose_transformed frame_id=%s", pose_in.header.frame_id.c_str(),
				//  pose_transformed.header.frame_id.c_str());
				// Set transformed attractor point
				manage_vt->set_attractor_point(
						pose_transformed.pose.position.x, pose_transformed.pose.position.y,
						atan2(2.0 * (pose_transformed.pose.orientation.w * pose_transformed.pose.orientation.z),
									1 - (2 * (pose_transformed.pose.orientation.z * pose_transformed.pose.orientation.z))));

				// transform mtt to /vehicle_odometry
				pcl::PointCloud<pcl::PointXYZ> pct;
				mtt::TargetListPC msg_transformed;

				pcl::PointCloud<pcl::PointXYZ> pct2;
				mtt::TargetListPC msg_transformed2;

				//--------------------------------------------------------------------------------------------------------//
				// ROS_INFO("pc_v1 size = %ld", pc_v1.size());
				for (size_t i = 0; i < pc_v1.size(); ++i)
				{
					if (i == 0)
					{
						try
						{
							p_listener->lookupTransform(pc_v1[i].header.frame_id, "/vehicle_odometry", ros::Time(0), transform_mtt);
							// ROS_INFO("FFFFFFrame_id=%s ", pc_v1[i].header.frame_id.c_str());
							mtt_broadcaster.sendTransform(
									tf::StampedTransform(transform_mtt, time, pc_v1[i].header.frame_id, "/vehicle_odometry"));
							ros::spinOnce();
						}
						catch (tf::TransformException ex)
						{
							ROS_ERROR("%s", ex.what());
						}
					}

					// ROS_INFO("pc_v1[%ld] frame_id=%s pcp frame_id=%s", i, pc_v1[i].header.frame_id.c_str(),
					//  pct.header.frame_id.c_str());
					pcl_ros::transformPointCloud(pc_v1[i], pct, transform_mtt.inverse());

					sensor_msgs::PointCloud2 pc_msg;
					pcl::toROSMsg(pct, pc_msg);
					pc_msg.header.frame_id = "/vehicle_odometry";
					pc_msg.header.stamp = ros::Time::now();
					msg_transformed.obstacle_lines.push_back(pc_msg);
				}
				manage_vt->set_obstacles(msg_transformed);

				// road lines
				if (_simulation_)
				{
					// ROS_INFO("pc_v2 size = %ld", pc_v2.size());
					for (size_t i = 0; i < pc_v2.size(); ++i)
					{
						if (i == 0)
						{
							try
							{
								p_listener->lookupTransform(pc_v2[i].header.frame_id, "/vehicle_odometry", ros::Time(0), transform_mtt);
								mtt_broadcaster.sendTransform(
										tf::StampedTransform(transform_mtt, time, pc_v2[i].header.frame_id, "/vehicle_odometry"));
								ros::spinOnce();
							}
							catch (tf::TransformException ex)
							{
								ROS_ERROR("%s", ex.what());
							}
						}
						pcl_ros::transformPointCloud(pc_v2[i], pct2, transform_mtt.inverse());
						sensor_msgs::PointCloud2 pc_msg2;
						pcl::toROSMsg(pct2, pc_msg2);
						pc_msg2.header.frame_id = "/vehicle_odometry";
						pc_msg2.header.stamp = ros::Time::now();
						msg_transformed2.obstacle_lines.push_back(pc_msg2);
					}
					manage_vt->set_lines(msg_transformed2);
				}

				//   ___________________________________
				//   |                                 |
				//   |        Draw                     |
				//   |_________________________________|
				manage_vt->create_static_markers();

				ros::Time st = ros::Time::now();

				manage_vt->compute_trajectories_scores();

				// change speed in function of steering angle
				if (manage_vt->chosen_traj.index != -1)
				{
					speed = angle_to_speed(manage_vt->chosen_traj.alpha);
					// ROS_INFO("speeeeeeeeeeeeed = %f", speed);
				}

				//   ___________________________________
				//   |                                 |
				//   |     Publish direction and speed |
				//   |_________________________________|
				geometry_msgs::Twist twist_msg;

				if (manage_vt->chosen_traj.index != -1)
				{
					// twist_msg.angular.z = compute_last_dir(manage_vt->chosen_traj.alpha);
					twist_msg.angular.z = compute_last_dir(manage_vt->chosen_traj.alpha);
					if (manage_vt->chosen_traj.score <= 0)
					{
						twist_msg.linear.x = 0;
					}
					else
					{
						twist_msg.linear.x = speed;
					}
					twist_pub.publish(twist_msg);
				}

				//   ___________________________________
				//   |                                 |
				//   |     Publish distance            |
				//   |_________________________________|

				std_msgs::Float64MultiArray mindist_msg;
				mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
				mindist_msg.layout.dim[0].label = "sim";
				mindist_msg.layout.dim[0].size = 2;
				mindist_msg.layout.dim[0].stride = 2;
				mindist_msg.layout.data_offset = 0;

				if (manage_vt->chosen_traj.index != -1)
				{
					vector<double> vec1 = { ros::Time::now().toSec(), manage_vt->chosen_traj.min_dist };
					mindist_msg.data.clear();
					mindist_msg.data.insert(mindist_msg.data.end(), vec1.begin(), vec1.end());
					mindist_pub.publish(mindist_msg);
				}

				//   ___________________________________
				//   |                                 |
				//   |     """"""""""""""""""""""""""" |
				//   |_________________________________|

				// cout<<"Compute scores: "<<(ros::Time::now()-st).toSec();

				ROS_INFO("manage_vt chosen traj= %f", manage_vt->chosen_traj.alpha);
				ROS_INFO("chosen traj min dist= %f", manage_vt->chosen_traj.min_dist);

				trajectory_planner::traj_info info;

				manage_vt->get_traj_info_msg_from_chosen(&info);

				commandPublisher.publish(info);

				visualization_msgs::MarkerArray marker_array;
				manage_vt->compute_vis_marker_array(&marker_array);
				array_pub.publish(marker_array);
				// cout<<"ja size:"<<marker_array.markers.size()<<endl;

				have_plan = true;
			}
		}

		if (have_plan == true)
		{
			// !!!! The previous 'if' must have a weight evaluation !!!!  if ... && GlobalScore>=0.74

			mw_broadcaster.sendTransform(tf::StampedTransform(transformw, ros::Time::now(), "/planning_world", "/vehicle_odometry"));
			// cout << "stat Publishing transform" << endl;
		}

		// printf("CURRENT NODE-> %d\n",node);
		// printf("Distance Travelled-> %f\n",base_status.distance_traveled);

		loop_rate.sleep();
		ros::spinOnce();
	}
}
#endif
