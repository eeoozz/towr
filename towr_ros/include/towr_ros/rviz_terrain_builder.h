#ifndef TOWR_ROS_RVIZ_TERRAIN_BUILDER_H_
#define TOWR_ROS_RVIZ_TERRAIN_BUILDER_H_

#include <string>

#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>


namespace towr {

/**
 * @brief  Constructs RVIZ markers that show different terrains.
 *
 * This class shows simple terrains (flat, sloped, stairs, ...) through
 * basic RVIZ shapes.
 */
class RvizTerrainBuilder {
public:
  using Marker       = visualization_msgs::Marker;
  using MarkerArray  = visualization_msgs::MarkerArray;
  using Vector3d     = Eigen::Vector3d;
  using Quat         = Eigen::Quaterniond;

  /**
   * @brief  Creates a default terrain builder object.
   */
  RvizTerrainBuilder () = default;
  virtual ~RvizTerrainBuilder () = default;

  /**
   * @brief  Constructs the rviz markers for a specific terrain.
   * @param  terrain_id  The identifier for a specific terrain.
   */
  MarkerArray BuildTerrain(int terrain_id);

private:
  MarkerArray BuildTerrainFlat()      const;
  MarkerArray BuildTerrainUnmodified()     const;
  MarkerArray BuildTerrainDoor()    const;
  MarkerArray BuildTerrainStairs()       const;
  MarkerArray BuildTerrainObstacles()     const;
  MarkerArray BuildTerrainNarrowAisle()   const;
  MarkerArray BuildTerrainSlope() const;
  MarkerArray BuildTerrainGap() const;

  Marker BuildTerrainBlock(const Vector3d& pos,
                           const Vector3d& edge_length,
                           const Quat& ori = Quat::Identity()) const;

  const double eps_ = 0.02;           // for lowering of terrain.
  const int terrain_ids_start_ = 50;  // to not overwrite other RVIZ markers.
  std::string rviz_frame_ = "world";  // the name of the frame set in RVIZ.
};

} /* namespace towr */

#endif /* TOWR_ROS_RVIZ_TERRAIN_BUILDER_H_ */

/*
#ifndef TOWR_ROS_RVIZ_TERRAIN_BUILDER_H_
#define TOWR_ROS_RVIZ_TERRAIN_BUILDER_H_

#include <string>

#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>


namespace towr {

/**
 * @brief  Constructs RVIZ markers that show different terrains.
 *
 * This class shows simple terrains (flat, sloped, stairs, ...) through
 * basic RVIZ shapes.
 *
class RvizTerrainBuilder {
public:
  using Marker       = visualization_msgs::Marker;
  using MarkerArray  = visualization_msgs::MarkerArray;
  using Vector3d     = Eigen::Vector3d;
  using Quat         = Eigen::Quaterniond;

  /**
   * @brief  Creates a default terrain builder object.
   *
  RvizTerrainBuilder () = default;
  virtual ~RvizTerrainBuilder () = default;

  /**
   * @brief  Constructs the rviz markers for a specific terrain.
   * @param  terrain_id  The identifier for a specific terrain.
   *
  MarkerArray BuildTerrain(int terrain_id);

private:
  MarkerArray BuildTerrainFlat()      const;
  MarkerArray BuildTerrainBlock()     const;
  MarkerArray BuildTerrainStairs()    const;
  MarkerArray BuildTerrainGap()       const;
  MarkerArray BuildTerrainSlope()     const;
  MarkerArray BuildTerrainChimney()   const;
  MarkerArray BuildTerrainChimneyLR() const;

  Marker BuildTerrainBlock(const Vector3d& pos,
                           const Vector3d& edge_length,
                           const Quat& ori = Quat::Identity()) const;

  const double eps_ = 0.02;           // for lowering of terrain.
  const int terrain_ids_start_ = 50;  // to not overwrite other RVIZ markers.
  std::string rviz_frame_ = "world";  // the name of the frame set in RVIZ.
};

}

#endif
*/
