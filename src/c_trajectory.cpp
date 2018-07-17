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
 * @file c_trajectory.cpp
 * @brief c_trajectory class generator
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file c_trajectory.cpp
 * @brief c_trajectory class generator
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <trajectory_planner/c_trajectory.h>
#include <trajectory_planner/trajectory_planner_nodelet.h>

/**
 * @brief Test the input vectors and compute the local node coordinates
 * @param vector<double> alpha_in
 * @param vector<double> arc_in
 * @param vector<double> speed_in
 * @return t_func_output
 */
t_func_output c_trajectory::generate(vector<double> alpha_in, vector<double> arc_in, vector<double> speed_in,
																		 t_vehicle_description& vd)
{
	// ________________________________
	//|                                |
	//|       Test input vectors       |
	//|________________________________|
	if ((alpha_in.size() == arc_in.size()) && (arc_in.size() == speed_in.size()))
	{
		// Copy all the values to internal variables
		double arc_sum = 0;
		// clear before new assembly
		alpha.clear();
		arc.clear();
		speed.clear();
		total_arc.clear();
		for (size_t i = 0; i < alpha_in.size(); ++i)
		{
			if (abs(alpha_in[i]) <= M_PI / 2)
			{
				alpha.push_back(alpha_in[i]);
				arc.push_back(arc_in[i]);
				speed.push_back(speed_in[i]);
				arc_sum = arc_sum + arc[i];
				total_arc.push_back(arc_sum);
			}
			else
			{
				ROS_ERROR("Detected alpha bigger than pi/2");
				return FAILURE;
			}
		}

		// ________________________________
		//|                                |
		//|   Compute local node coords    |
		//|________________________________|
		// clear before new assembly
		lx.clear();
		ly.clear();
		ltheta.clear();
		for (size_t i = 0; i < alpha.size(); ++i)
		{
			lx.push_back((D / tan(alpha[i])) * sin(arc[i] / (D / tan(alpha[i]))));
			ly.push_back((D / tan(alpha[i])) - (D / tan(alpha[i])) * cos(arc[i] / (D / tan(alpha[i]))));
		}

		for (size_t i = 0; i < alpha.size(); ++i)
		{
			// Local coords of IRC
			double lcx = 0;
			double lcy = D / tan(alpha[i]);

			if (alpha[i] > 0)
			{
				ltheta.push_back(M_PI / 2 + atan2(ly[i] - lcy, lx[i] - lcx));
			}
			else
			{
				ltheta.push_back(-M_PI / 2 + atan2(ly[i] - lcy, lx[i] - lcx));
			}
		}

		// ________________________________
		//|                                |
		//|     Define transformations     |
		//|________________________________|
		// Compute transformation
		compute_transformation();

		//   ___________________________________
		//   |                                 |
		//   | Compute vhc lines for each node |
		//   |_________________________________|
		// clear before new assembly
		v_lines.clear();
		for (size_t i = 0; i <= alpha.size(); ++i)
		{
			std::vector<t_lines> v;
			t_lines line1;

			if (i == 0)
			{
				line1.x[0] = -vd.lenght_back * cos(0) - (vd.width / 2.0) * sin(0);
				line1.y[0] = -vd.lenght_back * sin(0) + (vd.width / 2.0) * cos(0);
				line1.x[1] = +vd.lenght_front * cos(0) - (vd.width / 2.0) * sin(0);
				line1.y[1] = +vd.lenght_front * sin(0) + (vd.width / 2.0) * cos(0);
				v.push_back(line1);

				line1.x[0] = +vd.lenght_front * cos(0) - (vd.width / 2.0) * sin(0);
				line1.y[0] = +vd.lenght_front * sin(0) + (vd.width / 2.0) * cos(0);
				line1.x[1] = +vd.lenght_front * cos(0) + (vd.width / 2.0) * sin(0);
				line1.y[1] = +vd.lenght_front * sin(0) - (vd.width / 2.0) * cos(0);
				v.push_back(line1);

				line1.x[0] = +vd.lenght_front * cos(0) + (vd.width / 2.0) * sin(0);
				line1.y[0] = +vd.lenght_front * sin(0) - (vd.width / 2.0) * cos(0);
				line1.x[1] = -vd.lenght_back * cos(0) + (vd.width / 2.0) * sin(0);
				line1.y[1] = -vd.lenght_back * sin(0) - (vd.width / 2.0) * cos(0);
				v.push_back(line1);

				line1.x[0] = -vd.lenght_back * cos(0) + (vd.width / 2.0) * sin(0);
				line1.y[0] = -vd.lenght_back * sin(0) - (vd.width / 2.0) * cos(0);
				line1.x[1] = -vd.lenght_back * cos(0) - (vd.width / 2.0) * sin(0);
				line1.y[1] = -vd.lenght_back * sin(0) + (vd.width / 2.0) * cos(0);
				v.push_back(line1);
			}
			else
			{
				line1.x[0] = x[i - 1] - vd.lenght_back * cos(theta[i - 1]) - (vd.width / 2.0) * sin(theta[i - 1]);
				line1.y[0] = y[i - 1] - vd.lenght_back * sin(theta[i - 1]) + (vd.width / 2.0) * cos(theta[i - 1]);
				line1.x[1] = x[i - 1] + vd.lenght_front * cos(theta[i - 1]) - (vd.width / 2.0) * sin(theta[i - 1]);
				line1.y[1] = y[i - 1] + vd.lenght_front * sin(theta[i - 1]) + (vd.width / 2.0) * cos(theta[i - 1]);
				v.push_back(line1);

				line1.x[0] = x[i - 1] + vd.lenght_front * cos(theta[i - 1]) - (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[0] = y[i - 1] + vd.lenght_front * sin(theta[i - 1]) + (vd.width / 2.0) * cos(theta[i - 1]);
				line1.x[1] = x[i - 1] + vd.lenght_front * cos(theta[i - 1]) + (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[1] = y[i - 1] + vd.lenght_front * sin(theta[i - 1]) - (vd.width / 2.0) * cos(theta[i - 1]);
				v.push_back(line1);

				line1.x[0] = x[i - 1] + vd.lenght_front * cos(theta[i - 1]) + (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[0] = y[i - 1] + vd.lenght_front * sin(theta[i - 1]) - (vd.width / 2.0) * cos(theta[i - 1]);
				line1.x[1] = x[i - 1] - vd.lenght_back * cos(theta[i - 1]) + (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[1] = y[i - 1] - vd.lenght_back * sin(theta[i - 1]) - (vd.width / 2.0) * cos(theta[i - 1]);
				v.push_back(line1);

				line1.x[0] = x[i - 1] - vd.lenght_back * cos(theta[i - 1]) + (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[0] = y[i - 1] - vd.lenght_back * sin(theta[i - 1]) - (vd.width / 2.0) * cos(theta[i - 1]);
				line1.x[1] = x[i - 1] - vd.lenght_back * cos(theta[i - 1]) - (vd.width / 2.0) * sin(theta[i - 1]);
				;
				line1.y[1] = y[i - 1] - vd.lenght_back * sin(theta[i - 1]) + (vd.width / 2.0) * cos(theta[i - 1]);
				v.push_back(line1);
			}
			v_lines.push_back(v);
		}

		return SUCCESS;
	}
	else
	{
		ROS_ERROR("Input vector dimensions mismatch");
		return FAILURE;
	}
}

/**
 * @brief Compute the transformations
 * @param void
 * @return t_func_output
 */
t_func_output c_trajectory::compute_transformation()
{
	// clear before new assembly
	ltrans.clear();
	theta.clear();
	x.clear();
	y.clear();

	// compute vector of local transforms. Transform at position i transforms from i-1 to i
	pcl::PointCloud<pcl::PointXYZ> ponto_teste, ponto_result;
	theta.push_back(ltheta[0]);
	for (size_t i = 0; i < alpha.size(); ++i)
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(lx[i], ly[i], 0));
		transform.setRotation(tf::createQuaternionFromRPY(0, 0, ltheta[i]));
		ltrans.push_back(transform);

		pcl::PointXYZ pt1(0, 0, 0);
		ponto_teste.points.push_back(pt1);

		for (int j = i; j >= 0; --j)
		{
			pcl_ros::transformPointCloud(ponto_teste, ponto_result, ltrans[j]);

			if (j == 0)
			{
				x.push_back(ponto_result.points[0].x);
				y.push_back(ponto_result.points[0].y);
				ponto_result.clear();
				ponto_teste.clear();
			}
			else
			{
				ponto_teste.clear();
				ponto_teste.points.push_back(ponto_result.points[0]);
				ponto_result.clear();
			}
		}

		if (i != 0)
		{
			double theta_provi = theta[i - 1] + ltheta[i];
			while (theta_provi < -M_PI)
			{
				theta_provi = 2 * M_PI - abs(theta_provi);
			}
			while (theta_provi > M_PI)
			{
				theta_provi = theta_provi - 2 * M_PI;
			}
			theta.push_back(theta_provi);
		}
	}
	return SUCCESS;
}

/**
 * @brief Add markers to marker array
 * @param std::vector<visualization_msgs::Marker>* marker_vec
 * @param int* marker_count
 * @param int num_traj
 * @return void
 */
void c_trajectory::create_markers(std::vector<visualization_msgs::Marker>* marker_vec, int* marker_count, int num_traj)
{
	// Create a marker
	visualization_msgs::Marker marker, marker2;
	geometry_msgs::Point p, pp;
	std_msgs::ColorRGBA color;

	// ________________________________
	//|                                |
	//|           Line List            |
	//|________________________________|
	// Line marker to trajectory
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "lines";
	marker.id = num_traj;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.scale.x = 0.01;
	marker.color.r = 0.0;
	marker.color.g = 0.7;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	int first_step = 1;
	for (size_t i = 0; i < alpha.size(); ++i)
	{
		if (first_step == 1)
		{
			p.x = 0;
			p.y = 0;
			p.z = 0;
			marker.points.push_back(p);
			p.x = x[i];
			p.y = y[i];
			p.z = 0;
			marker.points.push_back(p);
			first_step = 0;
		}
		else
		{
			p.x = x[i - 1];
			p.y = y[i - 1];
			p.z = 0;
			marker.points.push_back(p);
			p.x = x[i];
			p.y = y[i];
			p.z = 0;
			marker.points.push_back(p);
		}
	}
	marker_vec->push_back(marker);

	// ________________________________
	//|                                |
	//|          Cube Marker           |
	//|________________________________|
	// Cube marker to t nodes
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "nodes";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.03;
	marker.scale.y = 0.03;
	marker.scale.z = 0.03;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	for (size_t i = 0; i < alpha.size(); ++i)
	{
		marker.id = (*marker_count)++;
		marker.pose.position.x = x[i];
		marker.pose.position.y = y[i];
		marker.pose.position.z = 0;
		marker_vec->push_back(marker);
	}

	// ________________________________
	//|                                |
	//|           text nodes           |
	//|________________________________|
	// Points marker to t nodes
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "node_number";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = 0.15;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.2;
	marker.color.a = 1.0;
	for (size_t i = 0; i < alpha.size(); ++i)
	{
		marker.id = (*marker_count)++;
		marker.pose.position.x = x[i];
		marker.pose.position.y = y[i];
		marker.pose.position.z = 0.075;
		char str[1024];
		sprintf(str, "%ld", i);
		marker.text = str;
		marker_vec->push_back(marker);
	}

	// ________________________________
	//|                                |
	//|         text trajectory        |
	//|________________________________|
	// Points marker to t nodes
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "trajectory_number";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = 0.2;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.id = (*marker_count)++;
	marker.pose.position.x = x[x.size() - 1];
	marker.pose.position.y = y[y.size() - 1];
	marker.pose.position.z = 0.2;
	marker.text = "Traj " + str(boost::format("%d") % (num_traj));
	marker_vec->push_back(marker);

	// ________________________________
	//|                                |
	//|             Arrow              |
	//|________________________________|
	// Line marker to trajectory
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "orientation_arrow";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.07;
	marker.scale.z = 0.07;
	for (size_t i = 0; i < alpha.size(); ++i)
	{
		marker.id = (*marker_count)++;
		marker.pose.position.x = x[i];
		marker.pose.position.y = y[i];
		marker.pose.position.z = 0;
		marker.pose.orientation.z = sin(theta[i] / 2);
		marker.pose.orientation.w = cos(theta[i] / 2);
		marker_vec->push_back(marker);
	}

	// ________________________________
	//|                                |
	//|     Rectangle (car shadow)     |
	//|________________________________|
	// Represents the form of the car in each node
	marker.header.frame_id = "/vehicle_odometry";
	marker.header.stamp = ros::Time::now();
	marker.ns = "car_shadow";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = _VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_;
	marker.scale.y = _VEHICLE_WIDTH_;
	marker.scale.z = 0.02;
	marker.color.r = 0.25;
	marker.color.g = 0.41;
	marker.color.b = 0.88;
	marker.color.a = 0.1;
	for (size_t i = 0; i <= alpha.size(); ++i)
	{
		marker.id = (*marker_count)++;
		if (i == 0)
		{
			marker.pose.position.x = ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * cos(0);
			marker.pose.position.y = ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * sin(0);
			marker.pose.position.z = 0;
			marker.pose.orientation.z = sin(0);
			marker.pose.orientation.w = cos(0);
		}
		else
		{
			marker.pose.position.x =
					x[i - 1] + ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * cos(theta[i - 1]);
			marker.pose.position.y =
					y[i - 1] + ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * sin(theta[i - 1]);
			marker.pose.position.z = 0;
			marker.pose.orientation.z = sin(theta[i - 1] / 2);
			marker.pose.orientation.w = cos(theta[i - 1] / 2);
		}
		marker_vec->push_back(marker);
	}

	// ________________________________
	//|                                |
	//|          Car contour           |
	//|________________________________|
	// Line marker to car contour
	marker2.header.frame_id = "/vehicle_odometry";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "car_contour";
	marker2.id = (*marker_count)++;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.type = visualization_msgs::Marker::LINE_LIST;
	marker2.scale.x = 0.01;
	marker2.color.r = 0.5;
	marker2.color.g = 0.5;
	marker2.color.b = 1.0;
	marker2.color.a = 0.5;
	for (size_t i = 0; i <= alpha.size(); ++i)
	{
		for (size_t l = 0; l < v_lines[i].size(); l++)
		{
			pp.x = v_lines[i][l].x[0];
			pp.y = v_lines[i][l].y[0];
			pp.z = 0;
			marker2.points.push_back(pp);

			pp.x = v_lines[i][l].x[1];
			pp.y = v_lines[i][l].y[1];
			pp.z = 0;
			marker2.points.push_back(pp);
		}
		marker_vec->push_back(marker2);
	}
}
