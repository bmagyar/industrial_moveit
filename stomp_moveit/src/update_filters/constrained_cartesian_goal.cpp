/*
 * constrained_cartesian_goal.cpp
 *
 *  Created on: Jun 3, 2016
 *      Author: Jorge Nicho
 */

#include <stomp_moveit/update_filters/constrained_cartesian_goal.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>
#include "stomp_moveit/utils/kinematics.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::ConstrainedCartesianGoal,stomp_moveit::update_filters::StompUpdateFilter);

static int CARTESIAN_DOF_SIZE = 6;

namespace stomp_moveit
{
namespace update_filters
{

ConstrainedCartesianGoal::ConstrainedCartesianGoal():
    name_("ConstrainedCartesianGoal")
{
  // TODO Auto-generated constructor stub

}

ConstrainedCartesianGoal::~ConstrainedCartesianGoal()
{
  // TODO Auto-generated destructor stub
}

bool ConstrainedCartesianGoal::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool ConstrainedCartesianGoal::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;

    XmlRpcValue dof_nullity_param = params["constrained_dofs"];
    XmlRpcValue dof_thresholds_param = params["cartesian_convergence"];
    XmlRpcValue joint_updates_param = params["joint_update_rates"];
    if((dof_nullity_param.getType() != XmlRpcValue::TypeArray) ||
        dof_nullity_param.size() < CARTESIAN_DOF_SIZE ||
        dof_thresholds_param.getType() != XmlRpcValue::TypeArray ||
        dof_thresholds_param.size() < CARTESIAN_DOF_SIZE  ||
        joint_updates_param.getType() != XmlRpcValue::TypeArray ||
        joint_updates_param.size() == 0)
    {
      ROS_ERROR("UnderconstrainedGoal received invalid array parameters");
      return false;
    }

    dof_nullity_.resize(CARTESIAN_DOF_SIZE);
    for(auto i = 0u; i < dof_nullity_param.size(); i++)
    {
      dof_nullity_(i) = static_cast<int>(dof_nullity_param[i]);
    }

    cartesian_convergence_thresholds_.resize(CARTESIAN_DOF_SIZE);
    for(auto i = 0u; i < dof_thresholds_param.size(); i++)
    {
      cartesian_convergence_thresholds_(i) = static_cast<double>(dof_thresholds_param[i]);
    }

    joint_update_rates_.resize(joint_updates_param.size());
    for(auto i = 0u; i < joint_updates_param.size(); i++)
    {
      joint_update_rates_(i) = static_cast<double>(joint_updates_param[i]);
    }

    max_iterations_ = static_cast<int>(params["max_ik_iterations"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConstrainedCartesianGoal::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();
  updated_parameters_.resize(num_joints,config.num_timesteps);
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state,*state_);

  const std::vector<moveit_msgs::Constraints>& goals = req.goal_constraints;
  if(goals.empty())
  {
    ROS_ERROR("A goal constraint was not provided");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // storing tool goal pose
  if(goals.front().position_constraints.empty() ||
      goals.front().orientation_constraints.empty())
  {
    ROS_WARN("A goal constraint for the tool link was not provided, using forward kinematics");

    // check joint constraints
    if(goals.front().joint_constraints.empty())
    {
      ROS_ERROR_STREAM("No joint values for the goal were found");
      error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    // compute FK to obtain tool pose
    const std::vector<moveit_msgs::JointConstraint>& joint_constraints = goals.front().joint_constraints;

    // copying goal values into state
    for(auto& jc: joint_constraints)
    {
      state_->setVariablePosition(jc.joint_name,jc.position);
    }

    state_->update(true);
    state_->enforceBounds(joint_group);

    // storing reference goal position tool and pose
    state_->copyJointGroupPositions(joint_group,ref_goal_joint_pose_);
    tool_goal_pose_ = state_->getGlobalLinkTransform(tool_link_);
  }
  else
  {
    // storing reference goal position and tool pose using ik
    const moveit_msgs::PositionConstraint& pos_constraint = goals.front().position_constraints.front();
    const moveit_msgs::OrientationConstraint& orient_constraint = goals.front().orientation_constraints.front();

    geometry_msgs::Pose pose;
    pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
    pose.orientation = orient_constraint.orientation;
    tf::poseMsgToEigen(pose,tool_goal_pose_);

    Eigen::VectorXd joint_pose = Eigen::VectorXd::Zero(num_joints);
    ref_goal_joint_pose_.resize(num_joints);
    robotStateMsgToRobotState(req.start_state,*state_);
    state_->copyJointGroupPositions(joint_group,joint_pose);
    if(!stomp_moveit::utils::kinematics::solveIK(state_,group_name_,dof_nullity_,
                                                 joint_update_rates_,cartesian_convergence_thresholds_,
                                                 max_iterations_,tool_goal_pose_,joint_pose,
                                                 ref_goal_joint_pose_))
    {

      ROS_WARN("%s IK failed using start pose as seed",getName().c_str());
      ref_goal_joint_pose_.resize(0);
    }
  }





  error_code.val = error_code.SUCCESS;
  return true;
}

bool ConstrainedCartesianGoal::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    const Eigen::MatrixXd& parameters,
                    Eigen::MatrixXd& updates,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace stomp_moveit::utils;

  // updating local parameters
  updated_parameters_ = parameters + updates;

  VectorXd init_joint_pose = updated_parameters_.rightCols(1);
  VectorXd joint_pose;

  filtered = false;

  if(!kinematics::solveIK(state_,group_name_,dof_nullity_,joint_update_rates_,cartesian_convergence_thresholds_,max_iterations_,
              tool_goal_pose_,init_joint_pose,joint_pose))
  {
    ROS_WARN("%s failed to find valid ik close to current goal pose",getName().c_str());

    if(ref_goal_joint_pose_.size() != 0)
    {
      joint_pose = ref_goal_joint_pose_; // assigning previously computed valid pose
      filtered = true;
      updates.rightCols(1) = joint_pose - parameters.rightCols(1);
    }

  }
  else
  {
    filtered = true;
    updates.rightCols(1) = joint_pose - parameters.rightCols(1);
  }

  return true;
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
