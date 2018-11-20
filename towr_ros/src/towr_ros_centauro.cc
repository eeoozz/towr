/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/initialization/quadruped_gait_generator.h>
#include <towr/models/examples/centauro_model.h>
#include <towr/terrain/examples/height_map_quad.h>
#include <towr/variables/euler_converter.h>

#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>


namespace towr {


TowrRos::TowrRos ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRos::UserCommandCallback, this);

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                          (xpp_msgs::robot_trajectory_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  model_.dynamic_model_   = std::make_shared<CentauroDynamicModel>();
  model_.kinematic_model_ = std::make_shared<CentauroKinematicModel>();
  gait_                   = std::make_shared<QuadrupedGaitGenerator>();

  // initial state
  BaseState b;
  b.lin.at(kPos).z() = 0.85;
  std::vector<Eigen::Vector3d> ee_pos(model_.kinematic_model_->GetNumberOfEndeffectors());

  ee_pos.at(LF) << 0.32,0.2224,0.0;
  ee_pos.at(RF) << 0.32,-0.2224,0;
  ee_pos.at(LH) << -0.32,0.2224,0;
  ee_pos.at(RH) << -0.32,-0.2224,0;

  towr_.SetInitialState(b, ee_pos);

  terrain_ = std::make_shared<FlatGround>();

  // could also use SNOPT here
  solver_ = std::make_shared<ifopt::Ipopt>();
  solver_->print_level_ = 5;
  solver_->max_cpu_time_ = 10.0;
  solver_->use_jacobian_approximation_ = false;

  visualization_dt_ = 0.02;
}

void
TowrRos::UserCommandCallback(const TowrCommandMsg& msg)
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  Parameters params;
  params.t_total_ = msg.total_duration;
  int n_ee = model_.kinematic_model_->GetNumberOfEndeffectors();
  auto gait = static_cast<GaitGenerator::GaitCombos>(msg.gait_id);
  gait_->SetCombo(gait);
  for (int ee=0; ee<n_ee; ++ee) {
    params.ee_phase_durations_.push_back(gait_->GetPhaseDurations(msg.total_duration, ee));
    params.ee_in_contact_at_start_.push_back(gait_->IsInContactAtStart(ee));
  }

  auto terrain_id = static_cast<TerrainID>(msg.terrain_id);
  terrain_ = HeightMapFactory::MakeTerrain(terrain_id);

  towr_.SetParameters(goal, params, model_, terrain_);


  ROS_DEBUG_STREAM("publishing robot parameters to " << robot_parameters_pub_.getTopic());
  xpp_msgs::RobotParameters robot_params_msg = BuildRobotParametersMsg(model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize) {
    towr_.SolveNLP(solver_);
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  // playback using terminal commands
  if (msg.replay_trajectory || msg.optimize) {

    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + xpp_msgs::terrain_info
        + " --quiet " + bag_file).c_str());
  }

  // publish entire trajectory directly
  if (msg.publish_traj) {
    ROS_DEBUG_STREAM("publishing optimized trajectory to " << cart_trajectory_pub_.getTopic());
    XppVec cart_traj = GetTrajectory();
    xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(cart_traj);
    cart_trajectory_pub_.publish(xpp_msg);
  }
}

std::vector<TowrRos::XppVec>
TowrRos::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<towr_.GetIterationCount(); ++iter) {
    towr_.SetSolution(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRos::XppVec
TowrRos::GetTrajectory () const
{
  SplineHolder solution = towr_.GetSolution();

  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {

    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {

      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRos::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRos::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void
TowrRos::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "towr_ros");
  towr::TowrRos towr_ros;
  ros::spin();

  return 1;
}
