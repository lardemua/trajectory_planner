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
#ifndef _c_manage_trajectory_H_
#define _c_manage_trajectory_H_

/**
 * @file c_manage_trajectory.h
 * @brief class which evaluates the trajectories planned
 * @author Joel Pereira
 * @version v0
 * @date 2012-05-10
 */

/**
 * @file c_manage_trajectory.h
 * @brief Class which evaluates the trajectories planned
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

// INCLUDES
#include <geometry_msgs/Polygon.h>
#include <math.h>
#include <mtt/TargetListPC.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <trajectory_planner/c_trajectory.h>
#include <trajectory_planner/traj_info.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

typedef struct
{
  // Positions
  double x;
  double y;
  // Orientation
  double theta;
} t_desired_coordinates;

typedef struct
{
  std::vector<double> x;
  std::vector<double> y;
  int id;
} t_obstacle;

// Defines
#define DONT_INTERSECT 0
#define DO_INTERSECT 1
#define COLLINEAR 2
#define _SPEED_SAFFETY_ 1
#define _SPEED_REQUIRED_ 10
#define _USE_MATH_DEFINES
#define _TRAJECTORY_LIB_DEBUG_ 1

// Trajectory weigths
#define W_DAP 0.10
#define W_ADAP 0.00
#define W_DLO 0.90
#define W_CL 0.40

// Namespaces
using namespace std;

class c_manage_trajectory
{
public:
  c_manage_trajectory(void)
  {
    chosen_traj.index = -1;
    AP.x = -1.0;
    AP.y = 0.0;
    AP.theta = M_PI / 8;
    current_node = 0;
    chosen_traj.min_dist = 0;
    chosen_traj.score = 0;
#if _TRAJECTORY_LIB_DEBUG_
    printf("Constructor...\n");
#endif
  };

  ~c_manage_trajectory()
  {
#if _TRAJECTORY_LIB_DEBUG_
    printf("Destructor...\n");
#endif
  };

  // -----  Variables  ------
  std::vector<c_trajectoryPtr> vt;  // the traj vector
  t_vehicle_description vehicle_description;
  t_desired_coordinates AP;
  double inter_axis_distance;
  int current_node;
  std::vector<visualization_msgs::Marker> static_marker_vec;
  std::vector<t_obstacle> vo;
  std::vector<t_obstacle> vl;
  struct
  {
    int index;
    double alpha;
    double min_dist;
    double score;
  } chosen_traj;

  // ------  Functions  ------
  t_func_output set_speed_vector(trajectory_planner::traj_info* info);
  t_func_output get_traj_info_msg_from_chosen(trajectory_planner::traj_info* info);
  t_func_output set_vehicle_description(double w, double lb, double lf, double ht, double hb);
  t_func_output set_obstacles(mtt::TargetListPC& msg);
  t_func_output set_lines(mtt::TargetListPC& msg);
  int lineSegmentIntersection(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double Dx, double Dy,
                              double* X, double* Y);
  t_func_output compute_DLO(c_trajectoryPtr& trajectory, std::vector<t_obstacle>& vo);
  t_func_output compute_trajectories_scores();
  t_func_output compute_vis_marker_array(visualization_msgs::MarkerArray* marker_array);
  t_func_output create_static_markers();
  t_func_output create_new_trajectory(vector<double> alpha_in, vector<double> arc_in, vector<double> speed_in);
  t_func_output set_inter_axis_distance(double val);
  t_func_output compute_DAP(c_trajectoryPtr& trajectory, t_desired_coordinates& AP);
  void draw_on_node(c_trajectoryPtr& trajectory, std::vector<visualization_msgs::Marker>* marker_vec, int* marker_count,
                    double z_high, double value, double normalized_value, string string_init);
  double compute_ADAP(c_trajectoryPtr& trajectory, t_desired_coordinates& AP, int i);
  t_func_output set_attractor_point(double x, double y, double theta);
  t_func_output compute_global_traj_score(c_trajectoryPtr& trajectory);
  t_func_output compute_chosen_traj();
  int set_chosen_traj(int n);
  int wn_PnPoly(geometry_msgs::Point32 P, geometry_msgs::Polygon* V, int n);
  inline int isLeft(geometry_msgs::Point32 P0, geometry_msgs::Point32 P1, geometry_msgs::Point32 P2);
  t_func_output update_trajectory(vector<double> alpha_in, vector<double> arc_in, vector<double> speed_in,
                                  int traj_num);

private:
};

typedef boost::shared_ptr<c_manage_trajectory> c_manage_trajectoryPtr;
#endif
