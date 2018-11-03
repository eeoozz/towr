

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_QUAD_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_QUAD_H_

#include <towr/terrain/height_map.h>

namespace towr {

/**
 * @brief Terrains IDs corresponding to a draw function in xpp_vis and a
 * detailed (gradient supplying) function in the optimizer.
 */
enum TerrainID { FlatID=0,
                 UnmodifiedID,
                 DoorID,
                 StairsID,
                 ObstaclesID,
                 NarrowAisleID, //1.25*width of the robot
                 GapID, //30 cm apart
                 SlopeID, //30 degree
                 K_TERRAIN_COUNT };

/**
 * @brief Generates some predefined height profiles according to the ID.
 */
class HeightMapFactory {
public:
  static HeightMap::Ptr MakeTerrain(TerrainID type);
};


class FlatGround : public HeightMap {
public:
  FlatGround(double height = 0.0);
  virtual double GetHeight(double x, double y)  const override { return height_; };

private:
  double height_; // [m]
};

class UnmodifiedGround : public HeightMap  {
public:
  virtual double GetHeight(double x, double y) const override;
private:
  double height_ = 0;
};

class Door : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;
private:
  double width_ = 1.6;
  double length_ = 0.2;
  double dist_from_start = 3;
  double slope_ = 20;

  double hw = width_/2;
};

class Stairs : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;

private:
  double first_step_start_ = 2.5;
  double step_width_ = 0.2;
  double step_height_ = 0.2;
  int step_num_ = 5;
};

class Obstacles : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
private:
  double first_obs_dist_ = 2;
  double first_obs_width_ = 3.0;
  double first_obs_height_ = 0.25;
  double first_obs_length_ = 0.3;
  double first_obs_va_ = -0.3; //the leftmost edge of the block's variation from y = 0;

  double second_obs_dist_ = 3;
  double second_obs_width_ = 3.0;
  double second_obs_height_ = 0.25;
  double second_obs_length_ = 0.3;
  double second_obs_va_ = 0.3;
};

class NarrowAisle : public HeightMap  {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;
private:
  double aisle_width_ = 1.5;
  double slope_ = 20; //treat the wall as slope at both sides, 0.1m discrepancy at 2m height

  double hw = aisle_width_/2.0;
};

class Gap : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 2.4;//2.5
  const double w = 0.5;//0.3
  const double h = 1.5;

  const double slope_ = h/w;
  const double dx = w/2.0;
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;

  // generated with matlab
  // see matlab/gap_height_map.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4*h)/(w*w);
  const double b = -(8*h*xc)/(w*w);
  const double c = -(h*(w - 2*xc)*(w + 2*xc))/(w*w);
};

class Slope : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 3.5;
  const double up_length_   = 3.464;
  const double height_center_ = 2;

  const double slope_end_ = slope_start_+up_length_;
  const double slope_ = height_center_/up_length_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_QUAD_H_ */
