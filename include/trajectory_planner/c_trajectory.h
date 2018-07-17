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
#ifndef _c_trajectory_H_
#define _c_trajectory_H_

/**
 * @file c_trajectory.h
 * @brief class which generates a few number of trajectories from an input file
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file c_trajectory.h
 * @brief Class which generates a few number of trajectories from the defines _NUM_TRAJ_, _NUM_NODES_ and
 * _TRAJECTORY_ANGLE_
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

// INCLUDES
#include <math.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

typedef enum { SUCCESS, FAILURE } t_func_output;

// Defines
#define PFLN printf("FILE %s LINE %d\n", __FILE__, __LINE__);
#define _USE_MATH_DEFINES
#define _TRAJECTORY_LIB_DEBUG_ 1

// Namespaces
using namespace std;

typedef struct
{
  double width;
  double lenght_back;
  double lenght_front;
  double height_top;
  double height_bottom;
} t_vehicle_description;

typedef struct
{
  double x[2];
  double y[2];
} t_lines;

typedef struct
{
  double x;
  double y;
} t_point;

class c_trajectory
{
public:
  c_trajectory(double D_in)
  {
    D = D_in;
#if _TRAJECTORY_LIB_DEBUG_
    printf("Constructor - Defining D...\n");
#endif
  };

  ~c_trajectory()
  {
#if _TRAJECTORY_LIB_DEBUG_
    printf("Destructor...\n");
#endif
  };

  // -----  Variables  ------
  // Local coordinates
  vector<vector<t_lines> > v_lines;
  vector<t_point> collision_pts;
  vector<double> lx;
  vector<double> ly;
  vector<double> ltheta;
  vector<tf::Transform> ltrans;

  // Global coordinates
  vector<double> x;
  vector<double> y;
  vector<double> theta;

  struct
  {
    double DAP;   // distance to application point
    double ADAP;  // angular difference related to the application point
    double DLO;
    double DAPnorm;   // normalized
    double ADAPnorm;  // normalized
    double DLOnorm;
    double FS;
    double CL;
    double overall_norm;
    double EVAL;  //
  } score;

  int closest_node;

  // Input parameters
  vector<double> alpha;
  vector<double> arc;
  vector<double> speed;
  double D;

  // Other paramenters
  vector<double> total_arc;

  // ------  Functions  ------
  t_func_output generate(vector<double> alpha_in, vector<double> arc_in, vector<double> speed_in,
                         t_vehicle_description& vd);
  t_func_output compute_transformation();
  void create_markers(std::vector<visualization_msgs::Marker>* marker_vec, int* marker_count, int num_traj);

private:
};
typedef boost::shared_ptr<c_trajectory> c_trajectoryPtr;

#endif
