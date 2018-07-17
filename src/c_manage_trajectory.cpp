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
 * @file c_manage_trajectory.cpp
 * @brief c_manage_trajectory class manager
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file c_manage_trajectory.cpp
 * @brief c_manage_trajectory class manager
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <trajectory_planner/c_manage_trajectory.h>
#include <trajectory_planner/trajectory_planner_nodelet.h>

/**
 * @brief Compute the free space. Determines the inclusion of a point in the rectangle that represents the car geometry.
 * @param trajectory_planner
 * @param vo
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_DLO(c_trajectoryPtr& trajectory, std::vector<t_obstacle>& vo)
{
  // delete all previous computed collision pts
  // trajectory->collision_pts.erase(trajectory->collision_pts.begin(), trajectory->collision_pts.end());

  if (trajectory->closest_node < 0 || trajectory->closest_node >= (int)trajectory->x.size())
  {
    ROS_ERROR("Error on node");
    return FAILURE;
  }
  // cycle all nodes until the closest node

  trajectory->score.DLO = 10.0;  // Equal to maximum_admissible_to_DLO
  trajectory->score.FS = 1;
  trajectory->score.CL = 1;
  trajectory->score.EVAL = 10.0;
  for (int n = 0; n <= trajectory->closest_node; ++n)
  {
    if (trajectory->v_lines.size() - 1 != trajectory->x.size())
    {
      ROS_ERROR("Node lines and number of nodes not equal");
    }

    // polygon to calculate intersections
    geometry_msgs::Polygon car_polygon;
    for (size_t l = 0; l < trajectory->v_lines[n].size(); ++l)
    {
      geometry_msgs::Point32 point;
      point.x = trajectory->v_lines[n][l].x[0];
      point.y = trajectory->v_lines[n][l].y[0];
      point.z = 0;  // pointPcl.z;
      car_polygon.points.push_back(point);
      if (l == (trajectory->v_lines[n].size() - 1))
      {
        point.x = trajectory->v_lines[n][l].x[1];
        point.y = trajectory->v_lines[n][l].y[1];
        point.z = 0;  // pointPcl.z;
        car_polygon.points.push_back(point);
      }
    }
    int car_polygon_size = car_polygon.points.size() - 1;
    // ROS_INFO("polygon size = %ld", car_polygon.points.size());

    // cycle all vehicle lines
    for (size_t l = 0; l < trajectory->v_lines[n].size(); ++l)
    {
      double Ax = trajectory->v_lines[n][l].x[0];
      double Ay = trajectory->v_lines[n][l].y[0];
      double Bx = trajectory->v_lines[n][l].x[1];
      double By = trajectory->v_lines[n][l].y[1];

      // cycle all obstacles
      // ROS_INFO("vo size = %ld", vo.size());
      for (size_t o = 0; o < vo.size(); ++o)
      {
        // cycle all lines inside each obstacle
        for (size_t lo = 0; lo < vo[o].x.size(); ++lo)
        {
          double DLOprev = sqrt(pow(trajectory->v_lines[n][l].x[0] - vo[o].x[lo], 2) +
                                pow(trajectory->v_lines[n][l].y[0] - vo[o].y[lo], 2));
          if (trajectory->score.DLO > DLOprev)
          {
            trajectory->score.DLO = DLOprev;
          }
          // simulator evaluation
          if (n == 0 && trajectory->score.EVAL > DLOprev)
          {
            trajectory->score.EVAL = DLOprev;
          }

          geometry_msgs::Point32 P;
          P.x = vo[o].x[lo];
          P.y = vo[o].y[lo];
          P.z = 0;  // pointPcl.z;

          int wn = wn_PnPoly(P, &car_polygon, car_polygon_size);

          if (wn != 0)
          {
            t_point p;
            p.x = P.x;
            p.y = P.y;
            // trajectory->collision_pts.push_back(p);
            trajectory->score.FS *= 0;
          }
        }
      }

      // road lines
      if (_simulation_)
      {
        // ROS_INFO("vl size = %ld", vl.size());
        for (size_t oo = 0; oo < vl.size(); ++oo)
        {
          for (size_t ll = 0; ll < vl[oo].x.size(); ++ll)
          {
            geometry_msgs::Point32 P;
            P.x = vl[oo].x[ll];
            P.y = vl[oo].y[ll];
            P.z = 0;  // pointPcl.z;

            int wn = wn_PnPoly(P, &car_polygon, car_polygon_size);

            if (wn != 0)
            {
              t_point p;
              p.x = P.x;
              p.y = P.y;
              if (trajectory->alpha[0] >= 0)
              {
                trajectory->score.CL = W_CL;
              }
            }
          }
        }
      }
    }
  }
  return SUCCESS;
}

/**
 * @brief Winding number test for a point in a polygon
 * @param P = a point
 * @param V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
 * @param n = number of points
 * @return wn = the winding number (=0 only when P is outside)
 */
int c_manage_trajectory::wn_PnPoly(geometry_msgs::Point32 P, geometry_msgs::Polygon* V, int n)
{
  int wn = 0;  // the  winding number counter

  // loop through all edges of the polygon
  for (int i = 0; i < n; i++)
  {  // edge from V[i] to  V[i+1]
    if (V->points[i].y <= P.y)
    {                                                       // start y <= P.y
      if (V->points[i + 1].y > P.y)                         // an upward crossing
        if (isLeft(V->points[i], V->points[i + 1], P) > 0)  // P left of  edge
          ++wn;                                             // have  a valid up intersect
    }
    else
    {                                                       // start y > P.y (no test needed)
      if (V->points[i + 1].y <= P.y)                        // a downward crossing
        if (isLeft(V->points[i], V->points[i + 1], P) < 0)  // P right of  edge
          --wn;                                             // have  a valid down intersect
    }
  }
  return wn;
}

/**
 * @brief Tests if a point is Left|On|Right of an infinite line
 * @param P0, P1, and P2 = points
 * @return >0 for P2 left of the line through P0 and P1
           =0 for P2  on the line
           <0 for P2  right of the line
 */
inline int c_manage_trajectory::isLeft(geometry_msgs::Point32 P0, geometry_msgs::Point32 P1, geometry_msgs::Point32 P2)
{
  return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
}

/**
 * @brief Verifies intersections between lines
 *
 * @param Ax
 * @param Ay
 * @param Bx
 * @param By
 * @param Cx
 * @param Cy
 * @param Dx
 * @param Dy
 * @param X
 * @param Y
 * @return int
 */
int c_manage_trajectory::lineSegmentIntersection(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                                                 double Dx, double Dy, double* X, double* Y)
{
  double distAB, theCos, theSin, newX, ABpos;

  //  Fail if either line segment is zero-length.
  if (((Ax == Bx) && (Ay == By)) || ((Cx == Dx) && (Cy == Dy)))
    return DONT_INTERSECT;

  //  Fail if the segments share an end-point.
  if (((Ax == Cx) && (Ay == Cy)) || ((Bx == Cx) && (By == Cy)) || ((Ax == Dx) && (Ay == Dy)) ||
      ((Bx == Dx) && (By == Dy)))
  {
    return DONT_INTERSECT;
  }

  //  (1) Translate the system so that point A is on the origin.
  Bx -= Ax;
  By -= Ay;
  Cx -= Ax;
  Cy -= Ay;
  Dx -= Ax;
  Dy -= Ay;

  //  Discover the length of segment A-B.
  distAB = sqrt(Bx * Bx + By * By);

  //  (2) Rotate the system so that point B is on the positive X axis.
  theCos = Bx / distAB;
  theSin = By / distAB;
  newX = Cx * theCos + Cy * theSin;
  Cy = Cy * theCos - Cx * theSin;
  Cx = newX;
  newX = Dx * theCos + Dy * theSin;
  Dy = Dy * theCos - Dx * theSin;
  Dx = newX;

  //  Fail if segment C-D doesn't cross line A-B.
  if ((Cy < 0. && Dy < 0.) || (Cy >= 0. && Dy >= 0.))
    return DONT_INTERSECT;

  //  (3) Discover the position of the intersection point along line A-B.
  ABpos = Dx + (Cx - Dx) * Dy / (Dy - Cy);

  //  Fail if segment C-D crosses line A-B outside of segment A-B.
  if (ABpos < 0. || ABpos > distAB)
    return DONT_INTERSECT;

  //  (4) Apply the discovered position to line A-B in the original coordinate system.
  *X = Ax + ABpos * theCos;
  *Y = Ay + ABpos * theSin;

  //  Success.
  return DO_INTERSECT;
}

/**
 * @brief Sets the obstacles
 * @param mtt::TargetListPC& msg
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_obstacles(mtt::TargetListPC& msg)
{
  vo.erase(vo.begin(), vo.end());
  // ROS_INFO("msg_obstacles size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i)
  {
    t_obstacle o;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc);

    for (size_t j = 0; j < pc.points.size(); ++j)
    {
      o.x.push_back(pc.points[j].x);
      o.y.push_back(pc.points[j].y);
    }

    vo.push_back(o);
  }
  return SUCCESS;
}

/**
 * @brief Sets the obstacles
 * @param mtt::TargetListPC& msg
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_lines(mtt::TargetListPC& msg)
{
  vl.erase(vl.begin(), vl.end());
  // ROS_INFO("msg_lines size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i)
  {
    t_obstacle o;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc);

    for (size_t j = 0; j < pc.points.size(); ++j)
    {
      o.x.push_back(pc.points[j].x);
      o.y.push_back(pc.points[j].y);
    }

    vl.push_back(o);
  }
  return SUCCESS;
}

/**
 * @brief Set the vehicle description
 *
 * @param w
 * @param lb
 * @param lf
 * @param ht
 * @param hb
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_vehicle_description(double w, double lb, double lf, double ht, double hb)
{
  vehicle_description.width = w;
  vehicle_description.lenght_back = lb;
  vehicle_description.lenght_front = lf;
  vehicle_description.height_top = ht;
  vehicle_description.height_bottom = hb;
  return SUCCESS;
}

/**
 * @brief Gets the info of chosen nodes
 *
 * @param info
 * @return t_func_output
 */
t_func_output c_manage_trajectory::get_traj_info_msg_from_chosen(trajectory_planner::traj_info* info)
{
  info->arc.erase(info->arc.begin(), info->arc.end());
  info->total_arc.erase(info->total_arc.begin(), info->total_arc.end());
  info->alpha.erase(info->alpha.begin(), info->alpha.end());
  info->speed.erase(info->speed.begin(), info->speed.end());
  for (int j = 0; j <= (int)vt[chosen_traj.index]->closest_node; ++j)
  {
    info->arc.push_back(vt[chosen_traj.index]->arc[j]);
    info->total_arc.push_back(vt[chosen_traj.index]->total_arc[j]);
    info->alpha.push_back(vt[chosen_traj.index]->alpha[j]);
  }
  set_speed_vector(info);

  // cout<<"ARC SIZE (when constructing) "<<info->arc.size()<<endl;
  // cout<<"TOTAL ARC SIZE (when constructing) "<<info->total_arc.size()<<endl;
  // cout<<"ALPHA SIZE (when constructing) "<<info->alpha.size()<<endl;
  // cout<<"SPEED SIZE (when constructing) "<<info->speed.size()<<endl;
  return SUCCESS;
}

/**
 * @brief Determine the speed vector
 *
 * @param info
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_speed_vector(trajectory_planner::traj_info* info)
{
  for (int i = 0; i <= (int)vt[chosen_traj.index]->closest_node; ++i)
  {
    if (i < ((int)vt[chosen_traj.index]->arc.size() - 1))
    {
      if ((info->arc[i]) * (info->arc[i + 1]) < 0.0)
      {
        info->speed.push_back((info->arc[i] / fabs(info->arc[i])) *
                              _SPEED_SAFFETY_);  // This is the speed set to reverse/forward or forward/reverse
      }
      else
      {
        info->speed.push_back((info->arc[i] / fabs(info->arc[i])) * _SPEED_REQUIRED_);
      }
    }
    else
    {
      info->speed.push_back((info->arc[i] / fabs(info->arc[i])) * _SPEED_REQUIRED_);
    }
  }
  return SUCCESS;
}

/**
 * @brief Set the chosen trajectory
 * @param int n
 * @return int
 */
int c_manage_trajectory::set_chosen_traj(int n)
{
  chosen_traj.index = n;
  return 1;
}

/**
 * @brief Set the attractor point
 * @param x
 * @param y
 * @param theta
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_attractor_point(double x, double y, double theta)
{
  AP.x = x;
  AP.y = y;
  AP.theta = theta;
  return SUCCESS;
}

/**
 * @brief Computes the trajectory global score
 * @param trajectory
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_global_traj_score(c_trajectoryPtr& trajectory)
{
  // double W_DAP = 0.40;
  // double W_ADAP = 0.35;
  // double W_DLO = 0.25;
  trajectory->score.overall_norm =
      (W_DAP * trajectory->score.DAPnorm + W_ADAP * trajectory->score.ADAPnorm + W_DLO * trajectory->score.DLOnorm) *
      trajectory->score.FS * trajectory->score.CL;
  // cout<<"Overallscore= "<<trajectory->score.overall_norm<<endl;

  return SUCCESS;
}

/**
 * @brief Set the inter-axis distance of the vehicle
 * @param val
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_inter_axis_distance(double val)
{
  inter_axis_distance = val;
  return SUCCESS;
}

/**
 * @brief Create a trajectory
 * @param alpha_in
 * @param arc_in
 * @param speed_in
 * @return t_func_output
 */
t_func_output c_manage_trajectory::create_new_trajectory(vector<double> alpha_in, vector<double> arc_in,
                                                         vector<double> speed_in)
{
  // allocate space for new traj
  c_trajectoryPtr t_ptr(new c_trajectory(inter_axis_distance));
  vt.push_back(t_ptr);

  // set the parameters of the traj
  return vt[vt.size() - 1]->generate(alpha_in, arc_in, speed_in, vehicle_description);
}

/**
 * @brief Update a trajectory during planning execution
 * @param alpha_in
 * @param arc_in
 * @param speed_in
 * @param traj_num
 * @return t_func_output
 */
t_func_output c_manage_trajectory::update_trajectory(vector<double> alpha_in, vector<double> arc_in,
                                                     vector<double> speed_in, int traj_num)
{
  // set the parameters of the traj
  return vt[traj_num]->generate(alpha_in, arc_in, speed_in, vehicle_description);
}

/**
 * @brief Chooses the trajectory
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_chosen_traj(void)
{
  double max_val = -1000;
  chosen_traj.index = -1;

  for (int i = 0; i < (int)vt.size(); i++)
  {
    if (vt[i]->score.overall_norm > max_val)
    {
      chosen_traj.index = i;
      chosen_traj.min_dist = vt[i]->score.EVAL;
      chosen_traj.alpha = vt[i]->alpha[0];
      chosen_traj.score = vt[i]->score.overall_norm;
      max_val = vt[i]->score.overall_norm;
    }
  }

  if (chosen_traj.index != -1)
    return SUCCESS;
  else
    return FAILURE;
}

/**
 * @brief Create a static marker
 * @return t_func_output
 */
t_func_output c_manage_trajectory::create_static_markers(void)
{
  // cout<<"ja create static"<<endl;
  static_marker_vec.clear();
  int marker_count = 0;
  for (int i = 0; i < (int)vt.size(); i++)
  {
    vt[i]->create_markers(&static_marker_vec, &marker_count, i);
  }
  return SUCCESS;
}

/**
 * @brief Compute the markers array
 * @param  marker_array
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_vis_marker_array(visualization_msgs::MarkerArray* marker_array)
{
  std::vector<visualization_msgs::Marker> marker_vec;

  // ROS_INFO("static marker vec size=%ld", static_marker_vec.size());
  for (size_t i = 0; i < static_marker_vec.size(); ++i)
  {
    marker_vec.push_back(static_marker_vec[i]);
  }

  int marker_count = 0;
  for (int i = 0; i < (int)vt.size(); ++i)
  {
    // draw_on_node(vt[i], &marker_vec, &marker_count, 0.15, vt[i]->score.DAP, vt[i]->score.DAPnorm, "DAP ");
    // draw_on_node(vt[i], &marker_vec, &marker_count, 0.30, vt[i]->score.ADAP, vt[i]->score.ADAPnorm, "ADAP ");
    // draw_on_node(vt[i], &marker_vec, &marker_count, 0.45, vt[i]->score.DLO, vt[i]->score.DLOnorm, "DLO ");
    draw_on_node(vt[i], &marker_vec, &marker_count, 0.60,
                 (vt[i]->score.DAP + vt[i]->score.ADAP + vt[i]->score.DLO) * vt[i]->score.FS, vt[i]->score.overall_norm,
                 "P = ");
  }

  // ________________________________
  //|                                |
  //|           Line List            |
  //|________________________________|
  // Line marker to trajectory
  visualization_msgs::Marker marker2;
  geometry_msgs::Point p;
  marker2.header.frame_id = "/vehicle_odometry";
  marker2.header.stamp = ros::Time::now();
  marker2.ns = "chosen_trajectory";
  marker2.id = 0;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.type = visualization_msgs::Marker::LINE_LIST;
  marker2.scale.x = 0.08;
  marker2.color.r = 0.039;
  marker2.color.g = 0.619;
  marker2.color.b = 0.039;
  marker2.color.a = 1.00;
  int first_step = 1;
  for (size_t i = 0; i < vt[chosen_traj.index]->x.size(); ++i)
  {
    if (first_step == 1)
    {
      p.x = 0;
      p.y = 0;
      p.z = 2;
      marker2.points.push_back(p);
      p.x = vt[chosen_traj.index]->x[i];
      p.y = vt[chosen_traj.index]->y[i];
      p.z = 2;
      marker2.points.push_back(p);
      first_step = 0;
    }
    else
    {
      p.x = vt[chosen_traj.index]->x[i - 1];
      p.y = vt[chosen_traj.index]->y[i - 1];
      p.z = 2;
      marker2.points.push_back(p);
      p.x = vt[chosen_traj.index]->x[i];
      p.y = vt[chosen_traj.index]->y[i];
      p.z = 2;
      marker2.points.push_back(p);
    }
  }
  marker_vec.push_back(marker2);

  // ________________________________
  //|                                |
  //|     Rectangle (actual)         |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker5;
  marker5.header.frame_id = "/vehicle_odometry";
  marker5.header.stamp = ros::Time::now();
  marker5.ns = "car_actual_traj";
  marker5.id = 0;
  marker5.action = visualization_msgs::Marker::ADD;
  marker5.type = visualization_msgs::Marker::CUBE;
  marker5.scale.x = _VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_;
  marker5.scale.y = _VEHICLE_WIDTH_;
  marker5.scale.z = 0.001;
  marker5.color.r = 0;
  marker5.color.g = 1;
  marker5.color.b = 0;
  marker5.color.a = 0.1;
  for (size_t i = 0; i <= vt[chosen_traj.index]->x.size(); ++i)
  {
    if (i == 0)
    {
      marker5.pose.position.x = ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * cos(0);
      marker5.pose.position.y = ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) * sin(0);
      marker5.pose.position.z = -0.1;
      marker5.pose.orientation.z = sin(0);
      marker5.pose.orientation.w = cos(0);
    }
    else
    {
      marker5.pose.position.x = vt[chosen_traj.index]->x[i - 1] +
                                ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) *
                                    cos(vt[chosen_traj.index]->theta[i - 1]);
      marker5.pose.position.y = vt[chosen_traj.index]->y[i - 1] +
                                ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) *
                                    sin(vt[chosen_traj.index]->theta[i - 1]);
      marker5.pose.position.z = -0.1;
      marker5.pose.orientation.z = sin(vt[chosen_traj.index]->theta[i - 1] / 2);
      marker5.pose.orientation.w = cos(vt[chosen_traj.index]->theta[i - 1] / 2);
    }
    marker_vec.push_back(marker5);
    marker5.id += 1;
  }

  // ________________________________
  //|                                |
  //|        Best node (sphere)      |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "/vehicle_odometry";
  marker3.header.stamp = ros::Time::now();
  marker3.ns = "closest_node";
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.scale.x = 0.2;
  marker3.scale.y = 0.2;
  marker3.scale.z = 0.2;
  marker3.color.r = 1.0;
  marker3.color.g = 0.0;
  marker3.color.b = 0.1;
  marker3.color.a = 0.6;

  for (size_t i = 0; i < vt.size(); ++i)
  {
    marker3.id += 2;
    marker3.pose.position.x = vt[i]->x[vt[i]->closest_node];
    marker3.pose.position.y = vt[i]->y[vt[i]->closest_node];
    marker3.pose.position.z = 0;
    marker_vec.push_back(marker3);
  }

  // ________________________________
  //|                                |
  //|        Colision (cylinder)     |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker4;
  marker4.header.frame_id = "/vehicle_odometry";
  marker4.header.stamp = ros::Time::now();
  marker4.ns = "colision_points";
  marker4.action = visualization_msgs::Marker::ADD;
  marker4.type = visualization_msgs::Marker::CYLINDER;
  marker4.scale.x = 0.12;
  marker4.scale.y = 0.12;
  marker4.scale.z = 0.1;
  marker4.color.r = 1.0;
  marker4.color.g = 0.84;
  marker4.color.b = 0.0;
  marker4.color.a = 1.0;

  static size_t coli_marker_total = 0;
  int total = 0;
  for (size_t j = 0; j < vt.size(); ++j)
  {
    // ROS_INFO("Traj%ld has %ld collisions\n", j, vt[j]->collision_pts.size());

    for (size_t i = 0; i < vt[j]->collision_pts.size(); ++i)
    {
      marker4.id = total;
      marker4.pose.position.x = vt[j]->collision_pts[i].x;
      marker4.pose.position.y = vt[j]->collision_pts[i].y;
      marker_vec.push_back(marker4);
      total++;
    }
  }

  // ROS_INFO("total=%d coli=%ld", total, coli_marker_total);
  // erase old colision markers
  for (size_t j = total; j < coli_marker_total; ++j)
  {
    // ROS_INFO("deleting marker ");
    marker4.header.frame_id = "/vehicle_odometry";
    marker4.id = j;
    marker4.action = visualization_msgs::Marker::DELETE;
    marker_vec.push_back(marker4);
  }

  coli_marker_total = total;

  marker_array->markers = marker_vec;

  return SUCCESS;
}

/**
 * @brief Compute trajectories scores
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_trajectories_scores(void)
{
  double maximum_admissible_to_DAP = 8.0;   // ATENTION to negative values if bigger than maximum
  double maximum_admissible_to_DLO = 10.0;  // ATENTION to negative values if bigger than maximum
  for (int i = 0; i < (int)vt.size(); ++i)
  {
    // Compute DAP and ADAP
    compute_DAP(vt[i], AP);

    // Compute DLO
    compute_DLO(vt[i], vo);

    // normalize DAP
    vt[i]->score.DAPnorm = max(0.0, (1 - (vt[i]->score.DAP) / maximum_admissible_to_DAP));

    // normalize ADAP
    vt[i]->score.ADAPnorm = max(0.0, (1 - (vt[i]->score.ADAP / (M_PI))));

    // normalize DLO
    vt[i]->score.DLOnorm = (vt[i]->score.DLO) / maximum_admissible_to_DLO;
  }

  // compute overall score for each traj
  for (size_t i = 0; i < vt.size(); ++i)
  {
    compute_global_traj_score(vt[i]);
  }

  compute_chosen_traj();

  return SUCCESS;
}

/**
 * @brief Compute the distance to application point
 * @param trajectory
 * @param AP
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_DAP(c_trajectoryPtr& trajectory, t_desired_coordinates& AP)
{
  trajectory->score.DAP = 10e6;

  trajectory->closest_node = -1;

  for (size_t i = 0; i < trajectory->x.size(); ++i)
  {
    double DAP_prev = sqrt(pow(trajectory->x[i] - AP.x, 2) + pow(trajectory->y[i] - AP.y, 2));

    if (DAP_prev < trajectory->score.DAP)
    {
      trajectory->score.DAP = DAP_prev;
      trajectory->closest_node = i;
    }
  }

  if (trajectory->closest_node != -1)
  {
    trajectory->score.ADAP = compute_ADAP(trajectory, AP, trajectory->closest_node);
    return SUCCESS;
  }
  else
  {
    return FAILURE;
  }
}

/**
 * @brief Compute the angular difference
 * @param trajectory
 * @param AP
 * @param i
 * @return double
 */
double c_manage_trajectory::compute_ADAP(c_trajectoryPtr& trajectory, t_desired_coordinates& AP, int i)
{
  double adap = abs(trajectory->theta[i] - AP.theta);
  if (adap > M_PI)
    adap = 2 * M_PI - adap;
  return adap;
}

/**
 * @brief Draw some informations about trajectories
 * @param trajectory_planner
 * @param marker_vec
 * @param marker_count
 * @param z_high
 * @param value
 * @param normalized_value
 * @param string_init
 */
void c_manage_trajectory::draw_on_node(c_trajectoryPtr& trajectory, std::vector<visualization_msgs::Marker>* marker_vec,
                                       int* marker_count, double z_high, double value, double normalized_value,
                                       string string_init)
{
  // Create a marker
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;
  // ________________________________
  //|                                |
  //|           text nodes           |
  //|________________________________|
  // Points marker to t nodes
  marker.header.frame_id = "/vehicle_odometry";
  marker.header.stamp = ros::Time::now();
  marker.ns = "info";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.6;
  marker.color.r = 0.039;
  marker.color.g = 0.207;
  marker.color.b = 0.619;
  marker.color.a = 1.0;
  marker.id = (*marker_count)++;
  marker.pose.position.x =
      trajectory->x[trajectory->x.size() - 1] +
      (_VEHICLE_LENGHT_FRONT_ + (_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) *
          cos(trajectory->theta[trajectory->theta.size() - 1]);
  marker.pose.position.y =
      trajectory->y[trajectory->y.size() - 1] +
      (_VEHICLE_LENGHT_FRONT_ + (_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 - _VEHICLE_LENGHT_BACK_) *
          sin(trajectory->theta[trajectory->theta.size() - 1]);
  marker.pose.position.z = z_high;

  marker.text = string_init + str(boost::format("%.2f") % normalized_value);
  // string_init + str(boost::format("%.2f") % value) + " (" + str(boost::format("%.2f") % normalized_value) + ")";
  marker_vec->push_back(marker);
}
