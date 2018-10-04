
#include <towr/terrain/examples/height_map_quad.h>

namespace towr {

HeightMap::Ptr
HeightMapFactory::MakeTerrain (TerrainID type)
{
  switch (type) {
    case FlatID:      return std::make_shared<FlatGround>(); break;
    case DoorID:     return std::make_shared<Door>(); break;
    case StairsID:    return std::make_shared<Stairs>(); break;
    case ObstaclesID:   return std::make_shared<Obstacles>(); break;
    case NarrowAisleID: return std::make_shared<NarrowAisle>(); break;
    case GapID:       return std::make_shared<Gap>(); break;
    case SlopeID:     return std::make_shared<Slope>(); break;
    default: assert(false); break;
  }
}

FlatGround::FlatGround(double height)
{
  height_ = height;
}

//UNMODIFIED
double
UnmodifiedGround::GetHeight(double x, double y) const
{
  return 0.0;
}

//DOOR
double
Door::GetHeight(double x, double y) const
{
  double h = 0.0;

  if ((y< -hw) && (x> dist_from_start) && (x< dist_from_start+length_))
    h = (-hw - y) * slope_;

  if ((y> hw) && (x> dist_from_start) && (x< dist_from_start+length_))
    h = (y-hw) * slope_;

  return h;
}

double Door::GetHeightDerivWrtY(double x, double y) const
{
  double dhdy = 0.0;

  if ((y< -hw) && (x> dist_from_start) && (x< dist_from_start+length_))
    dhdy = -slope_;

  if ((y> hw) && (x> dist_from_start) && (x< dist_from_start+length_))
    dhdy = slope_;

  return dhdy;
}

// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int count=0; count<step_num_; count++) {
    if (x>= (count*step_width_+first_step_start_))
      h = step_height_*(count+1);
  }

  return h;
}

//OBSTACLES
double
Obstacles::GetHeight(double x, double y) const
{
  double h = 0.0;

  if ((x> first_obs_dist_) && (x< first_obs_dist_+first_obs_length_) && (y> first_obs_va_) && (y< first_obs_va_+first_obs_width_))
    h = first_obs_height_;

  if ((x> second_obs_dist_) && (x< second_obs_dist+second_obs_length_) && (y> second_obs_va_) && (y< second_obs_va_+second_obs_width_))
    h = second_obs_height_;

  return h;
}

//AISLE
double
NarrowAisle::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (y< -hw)
    h = (-hw - y) * slope_;

  if (y> hw)
    h = (y-hw) * slope_;

  return h;
}

double
NarrowAisle::GetHeightDerivWrtY (double x, double y) const
{
  double dhdy = 0.0;

  if (y<-hw)
    dhdy = -slope_;

  if (y>hw)
    dhdy = slope_;

  return dhdy;
}

// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}

// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if ((x >= slope_start_) && (x < slope_end_))
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= slope_end_) {
    z = height_center_;
  }

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if ((x >= slope_start_) && (x < slope_end_))
    dzdx = slope_;

  if (x >= slope_end_)
    dzdx = 0.0;

  return dzdx;
}


} /* namespace towr */
