#include <towr_ros/rviz_terrain_builder.h>

#include <cmath>
#include <string>
#include <vector>

#include <xpp_states/convert.h>

#include <towr/terrain/examples/height_map_quad.h>

namespace towr {

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrain (int terrain)
{
  MarkerArray msg;

  switch (terrain) {
    case FlatID:        msg = BuildTerrainFlat(); break;
    case UnmodifiedID:  msg = BuildTerrainUnmodified(); break;
    case DoorID:        msg = BuildTerrainDoor(); break;
    case StairsID:      msg = BuildTerrainStairs(); break;
    case ObstaclesID:   msg = BuildTerrainObstacles(); break;
    case NarrowAisleID: msg = BuildTerrainNarrowAisle(); break;
    case GapID:         msg = BuildTerrainGap(); break;
    case SlopeID:       msg = BuildTerrainSlope(); break;
    default: return MarkerArray(); // terrain visualization not implemented
  }

  int id = terrain_ids_start_;
  for (Marker& m : msg.markers)
    m.id = id++;

  return msg;
}

RvizTerrainBuilder::Marker
RvizTerrainBuilder::BuildTerrainBlock (const Vector3d& pos,
                                       const Vector3d& edge_length,
                                       const Quat& ori) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = xpp::Convert::ToRos<geometry_msgs::Point>(pos);
  m.pose.orientation = xpp::Convert::ToRos(ori);
  m.scale            = xpp::Convert::ToRos<geometry_msgs::Vector3>(edge_length);
  m.ns = "terrain";
  m.header.frame_id = rviz_frame_;
  m.color.r  = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.a = 1.0;

  return m;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainFlat() const
{
  MarkerArray msg;

  for (int i=0; i<terrain_ids_start_; ++i) {
    msg.markers.push_back(BuildTerrainBlock(Vector3d::Ones(), Vector3d::Ones()));
    msg.markers.back().color.a = 0.0;
  }

  // one long path
  Vector3d size_start_end(12,8,0.1);//5,1,0.1
  Vector3d center0(3.5, 0.0, -0.05-eps_);
  msg.markers.at(0) = BuildTerrainBlock(center0, size_start_end);

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainUnmodified() const
{
  MarkerArray msg;

  for (int i=0; i<terrain_ids_start_; ++i) {
    msg.markers.push_back(BuildTerrainBlock(Vector3d::Ones(), Vector3d::Ones()));
    msg.markers.back().color.a = 0.0;
  }

  // one long path
  Vector3d size_start_end(12,8,0.1);//5,1,0.1
  Vector3d center0(3.5, 0.0, -0.05-eps_);
  msg.markers.at(0) = BuildTerrainBlock(center0, size_start_end);

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainDoor() const
{
  double width_ = 1.6;
  double length_ = 0.2;
  double dist_from_start = 3;
  double slope_ = 20;


  MarkerArray msg;
  double area_width = 8;

  Vector3d size0(12,area_width,0.1);
  Vector3d center0(3.5, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));


  Vector3d size(length_,(area_width-width_)/2,3);
  Vector3d center1(dist_from_start+length_/2, -width_/4-area_width/4, 1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  Vector3d center2(dist_from_start+length_/2, width_/4+area_width/4, 1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center2, size));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainStairs() const
{
  double first_step_start_ = 2.5;
  double step_width_ = 0.2;
  double step_height_ = 0.2;
  int step_num_ = 5;

  MarkerArray msg;
  double area_width = 8.0;

  Vector3d size0(12,area_width,0.1);
  Vector3d center0(3.5, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));

  for (int count = 0; count < step_num_; count++) {
    Vector3d size_temp(step_width_, area_width, (count+1)*step_height_);
    Vector3d center_temp(first_step_start_+ (count+0.5)*step_width_, 0.0, -eps_+ (0.5*count+0.5)*step_height_);
    msg.markers.push_back(BuildTerrainBlock(center_temp, size_temp));
  }

  Vector3d size1(12- first_step_start_- step_num_*step_width_, area_width, step_num_*step_height_);
  Vector3d center1(6+ 0.5*first_step_start_+ 0.5*step_num_*step_width_, 0.0, 0.5*step_num_*step_height_- eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size1));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainObstacles() const
{
  double first_obs_dist_ = 2;
  double first_obs_width_ = 3.0;
  double first_obs_height_ = 0.1;
  double first_obs_length_ = 0.1;
  double first_obs_va_ = -0.3; //the leftmost edge of the block's variation from y = 0;

  double second_obs_dist_ = 3;
  double second_obs_width_ = 3.0;
  double second_obs_height_ = 0.1;
  double second_obs_length_ = 0.1;
  double second_obs_va_ = 0.3;

  MarkerArray msg;
  double area_width = 8.0;

  //remove previous ones
  Vector3d size0(1,1,1);
  Vector3d center0(3.5, 0.0, 1.0);
  for (int count =0; count < 4; count++ ) {
    msg.markers.push_back(BuildTerrainBlock(center0, size0));
    msg.markers.back().color.a = 0.0;
  }

  //current blocks
  Vector3d size1(12,area_width,0.1);
  Vector3d center1(3.5, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size1));

  Vector3d size2(first_obs_length_, first_obs_width_, first_obs_height_);
  Vector3d center2(first_obs_dist_+ 0.5*first_obs_length_, first_obs_va_- 0.5*first_obs_width_, 0.5*first_obs_height_-eps_);
  msg.markers.push_back(BuildTerrainBlock(center2, size2));

  Vector3d size3(second_obs_length_, second_obs_width_, second_obs_height_);
  Vector3d center3(second_obs_dist_+ 0.5*second_obs_length_, second_obs_va_+ 0.5*second_obs_width_, 0.5*second_obs_height_-eps_);
  msg.markers.push_back(BuildTerrainBlock(center3, size3));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainNarrowAisle() const
{
  double aisle_width_ = 1.5;

  MarkerArray msg;
  double area_width = 8;

  //remove previous ones
  Vector3d size0(1,1,1);
  Vector3d center0(3.5, 0.0, 1.0);
  for (int count =0; count < 4; count++ ) {
    msg.markers.push_back(BuildTerrainBlock(center0, size0));
    msg.markers.back().color.a = 0.0;
  }

  //current blocks
  Vector3d size1(12,area_width,0.1);
  Vector3d center1(3.5, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size1));


  Vector3d size(12,(area_width-aisle_width_)/2,3);
  Vector3d center2(3.5, -aisle_width_/4-area_width/4, 1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center2, size));

  Vector3d center3(3.5, aisle_width_/4+area_width/4, 1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center3, size));


  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainGap() const
{
  const double gap_start_ = 2.5;
  const double w = 0.3;

  MarkerArray msg;
  double area_width = 8;

  //remove previous ones
  Vector3d size0(1,1,1);
  Vector3d center0(3.5, 0.0, 1.0);
  for (int count =0; count < 5; count++ ) {
    msg.markers.push_back(BuildTerrainBlock(center0, size0));
    msg.markers.back().color.a = 0.0;
  }

  //current blocks
  Vector3d size1(6- 0.5*w,area_width,3);
  Vector3d center1(-0.5+ 0.25*w, 0.0, -1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size1));

  Vector3d center2(5.8+ 0.25*w, 0.0, -1.5-eps_);
  msg.markers.push_back(BuildTerrainBlock(center2, size1));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainSlope() const
{
  MarkerArray msg;
  double area_width = 8.0;

  const double slope_start_ = 3.5;
  const double up_length_   = 3.464;
  const double height_center_ = 2;

  const double slope_end_ = slope_start_+up_length_;
  const double slope_ = height_center_/up_length_;

  //remove previous ones
  Vector3d size0(1,1,1);
  Vector3d center0(3.5, 0.0, 1.0);
  for (int count =0; count < 5; count++ ) {
    msg.markers.push_back(BuildTerrainBlock(center0, size0));
    msg.markers.back().color.a = 0.0;
  }

  //current blocks
  Vector3d size_start_end(6,area_width,0.04);
  Vector3d center1(0.5, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size_start_end));


  double roll = 0.0;
  double pitch = -atan(slope_);
  double yaw = 0.0;
  Eigen::Quaterniond ori =  xpp::GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = height_center_/sin(pitch);
  double ly = area_width;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  // slope up
  Vector3d center2(slope_start_+up_length_/2, 0.0, height_center_/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori));

  // flat end
  Vector3d flat_end(3.0, area_width, 0.04);
  Vector3d center_end(8.464, 0.0, 1.95-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, flat_end));


  return msg;
}

} /* namespace towr */

/*
#include <towr_ros/rviz_terrain_builder.h>

#include <cmath>
#include <string>
#include <vector>

#include <xpp_states/convert.h>

#include <towr/terrain/examples/height_map_examples.h>

namespace towr {

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrain (int terrain)
{
  MarkerArray msg;

  switch (terrain) {
    case FlatID:      msg = BuildTerrainFlat(); break;
    case BlockID:     msg = BuildTerrainBlock(); break;
    case StairsID:    msg = BuildTerrainStairs(); break;
    case GapID:       msg = BuildTerrainGap(); break;
    case SlopeID:     msg = BuildTerrainSlope(); break;
    case ChimneyID:   msg = BuildTerrainChimney(); break;
    case ChimneyLRID: msg = BuildTerrainChimneyLR(); break;
    default: return MarkerArray(); // terrain visualization not implemented
  }

  int id = terrain_ids_start_;
  for (Marker& m : msg.markers)
    m.id = id++;

  return msg;
}

RvizTerrainBuilder::Marker
RvizTerrainBuilder::BuildTerrainBlock (const Vector3d& pos,
                                       const Vector3d& edge_length,
                                       const Quat& ori) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = xpp::Convert::ToRos<geometry_msgs::Point>(pos);
  m.pose.orientation = xpp::Convert::ToRos(ori);
  m.scale            = xpp::Convert::ToRos<geometry_msgs::Vector3>(edge_length);
  m.ns = "terrain";
  m.header.frame_id = rviz_frame_;
  m.color.r  = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.a = 1.0;

  return m;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainFlat() const
{
  MarkerArray msg;

  for (int i=0; i<terrain_ids_start_; ++i) {
    msg.markers.push_back(BuildTerrainBlock(Vector3d::Ones(), Vector3d::Ones()));
    msg.markers.back().color.a = 0.0;
  }

  // one long path
  Vector3d size_start_end(5,1,0.1);//5,1,0.1
  Vector3d center0(1.5, 0.0, -0.05-eps_);
  msg.markers.at(0) = BuildTerrainBlock(center0, size_start_end);

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainBlock() const
{
  double block_start = 0.7;
  double length_     = 3.5;
  double height_     = 0.5; // [m]


  MarkerArray msg;
  double area_width = 3.0;
  double ground_thickness = 0.1;

  Vector3d size0(3,area_width,ground_thickness);
  Vector3d center0(0, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));



  Vector3d size(length_,area_width,height_+ground_thickness);
  Vector3d center1(size.x()/2 + block_start, 0.0, size.z()/2-eps_-ground_thickness);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  return msg;
}


RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainStairs() const
{
  double first_step_start = 1.0;
  double height_first_step = 0.2;
  double first_step_width = 0.4;
  double width_top = 1.0;


  MarkerArray msg;
  double area_width = 3.0;

  Vector3d size0(6.5,area_width,0.1);
  Vector3d center0(2.25, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));


  Vector3d size(first_step_width+width_top,area_width,height_first_step);
  Vector3d center1(size.x()/2 + first_step_start, 0.0, size.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  double height_second_step = 0.4;
  Vector3d size2(width_top,area_width,height_second_step);
  Vector3d pos2(first_step_start+first_step_width+size2.x()/2, 0.0, size2.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size2));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainGap() const
{
  MarkerArray msg;

  double gap_start = 1.0;
  double l_gap = 0.6;

  double lx = gap_start*2.0;
  double ly = 3.0;
  double lz = 2.04;


  Vector3d size0(4.5,1,0.04);
  Vector3d center0(1.25, 0.0, -lz-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));
  msg.markers.back().color.a = 0.0;

  Vector3d size(lx,ly,lz);
  Vector3d center1(0.0, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  Vector3d pos2(l_gap + lx, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainSlope() const
{
  MarkerArray msg;
  double area_width = 3.0;

  const double slope_start = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;
  const double x_down_start_ = slope_start+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope = height_center/up_length_;

  double length_start_end_ = 2.0; // [m]


  Vector3d size_start_end(2,area_width,0.04);
  Vector3d center0(-length_start_end_/2. + slope_start, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));


  double roll = 0.0;
  double pitch = -atan(slope);
  double yaw = 0.0;
  Eigen::Quaterniond ori =  xpp::GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = height_center/sin(pitch);
  double ly = area_width;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  // slope up
  Vector3d center1(slope_start+up_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));

  // slope_down
  Vector3d center2(x_down_start_+down_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));

  // flat end
  Vector3d center_end(length_start_end_/2.+x_flat_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));


  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainChimney() const
{
  MarkerArray msg;
  double area_width = 3.0;

  const double x_start_ = 1.0;
  const double length_  = 1.5;
  const double y_start_ = 0.5; // for rosbag was: 0.8  or  0.5; distance to start of slope from center at z=0
  const double slope    = 3;   // for rosbag was: 2    or  3

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  xpp::GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 4.0;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,area_width,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, -3*eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 1.0;

//  // slope_right
//  Vector3d center2(x_start_+length_/2, -y_start_-eps_, 0);
//  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
//  msg.markers.back().color.a = 0.8;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainChimneyLR() const
{
  MarkerArray msg;
  double area_width=3.0;

  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope    = 2;

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  xpp::GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 2.5;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,area_width,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 1.0;

  // slope_right
  Vector3d center2(center1.x()+length_, -y_start_-eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
  msg.markers.back().color.a = 1.0;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+2*length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

}
*/
