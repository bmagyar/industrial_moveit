/**
 * @file openvdb_valgrind.cpp
 * @brief This is used for benchmark testing openvdb
 *
 * @author Levi Armstrong
 * @date Jun 29, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <string.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <constrained_ik/moveit_interface/joint_interpolation_planner.h>
#include <constrained_ik/ConstrainedIKPlannerDynamicReconfigureConfig.h>
#include <industrial_collision_detection/collision_robot_industrial.h>
#include <fstream>
#include <time.h>
#include "collision_robot_openvdb.h"
#include <openvdb/tools/VolumeToSpheres.h>
#include <openvdb/math/Transform.h>


using namespace ros;
using namespace constrained_ik;
using namespace Eigen;
using namespace moveit::core;
using namespace std;

typedef boost::shared_ptr<collision_detection::CollisionPlugin> CollisionPluginPtr;

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "openvdb_valgrid_name");
  ros::NodeHandle pnh;
  openvdb::initialize();

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
     ros::console::notifyLoggerLevelsChanged();

  sleep(1);
  robot_model_loader::RobotModelLoaderPtr loader;
  robot_model::RobotModelPtr robot_model;
  string urdf_file_path, srdf_file_path;

  urdf_file_path = package::getPath("stomp_test_support") + "/urdf/test_kr210l150_simple.urdf";
  srdf_file_path = package::getPath("stomp_test_kr210_moveit_config") + "/config/test_kr210.srdf";

  ifstream ifs1 (urdf_file_path.c_str());
  string urdf_string((istreambuf_iterator<char>(ifs1)), (istreambuf_iterator<char>()));

  ifstream ifs2 (srdf_file_path.c_str());
  string srdf_string((istreambuf_iterator<char>(ifs2)), (istreambuf_iterator<char>()));

  robot_model_loader::RobotModelLoader::Options opts(urdf_string, srdf_string);
  loader.reset(new robot_model_loader::RobotModelLoader(opts));
  robot_model = loader->getModel();

  if (!robot_model)
  {
    ROS_ERROR_STREAM("Unable to load robot model from urdf and srdf.");
    return false;
  }
  double t;
  ros::Time start;

  // create planning scene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  //Now assign collision detection plugin
  collision_detection::CollisionPluginLoader cd_loader;
  std::string class_name = "IndustrialFCL";
  cd_loader.activate(class_name, planning_scene, true);

  // get collision robot
  collision_detection::CollisionRobotIndustrialConstPtr collision_robot = boost::dynamic_pointer_cast<const collision_detection::CollisionRobotIndustrial>(planning_scene->getCollisionRobot());

  // get robot state
  moveit::core::RobotState robot_state = planning_scene->getCurrentState();
  robot_state.updateCollisionBodyTransforms();


  // get baseline fcl distance call data
  collision_detection::DistanceRequest req;
  collision_detection::DistanceResult res;
  req.acm = &planning_scene->getAllowedCollisionMatrix();
  req.group_name = "manipulator_rail";
  t=0;
  ROS_ERROR("FCL, Test Distance queries");
  for(int i = 0; i < 10; ++i)
  {
    res.clear();
    start = ros::Time::now();
    collision_robot->distanceSelf(req, res, robot_state);
    t+=(ros::Time::now() - start).toSec();
  }
  ROS_ERROR("FCL, Links: %s to %s", res.minimum_distance.link_name[0].c_str(), res.minimum_distance.link_name[1].c_str());
  ROS_ERROR("FCL, Distance: %0.8f", res.minimum_distance.min_distance);
  ROS_ERROR("FCL, Distance Average Time Elapsed: %0.8f (sec)", t/10.0);

  // get baseline fcl collision call data
  moveit::core::RobotState robot_state_coll = robot_state;
  collision_detection::CollisionRequest req_coll;
  collision_detection::CollisionResult res_coll;
  req_coll.group_name = "manipulator_rail";
  t=0;
  ROS_ERROR("FCL, Test Distance queries");
  for(int i = 0; i < 10; ++i)
  {
    res.clear();
    robot_state_coll.setToRandomPositions();
    robot_state_coll.updateCollisionBodyTransforms();
    start = ros::Time::now();
    collision_robot->checkSelfCollision(req_coll, res_coll, robot_state_coll, planning_scene->getAllowedCollisionMatrix());
    t+=(ros::Time::now() - start).toSec();
  }
  ROS_ERROR("FCL, Collision Average Time Elapsed: %0.8f (sec)", t/10.0);

  // distance field testing
  distance_field::OpenVDBDistanceField df(0.02);

  // Add sphere for testing distance
  ROS_ERROR("Add Sphere #1, to distance field.");
  start = ros::Time::now();
  shapes::Sphere *sphere1 = new shapes::Sphere(0.5);
  Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
  pose1.translation() = Eigen::Vector3d(4, 4, 4);
  df.addShapeToField(sphere1, pose1, 0.5/0.02, 0.5/0.02);
  ROS_ERROR("Add Sphere #1, Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());

  // Add second sphere for testing distance
  ROS_ERROR("Add Sphere #2, to distance field.");
  start = ros::Time::now();
  shapes::Sphere *sphere2 = new shapes::Sphere(0.5);
  Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
  pose2.translation() = Eigen::Vector3d(4, 4, 8.5);
  df.addShapeToField(sphere2, pose2, 0.5/0.02, 0.5/0.02);
  ROS_ERROR("Add Sphere #2, Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());

  double dist;
  Eigen::Vector3f pick_point(4.0, 4.0, 4.4), gradient;

  t=0;
  for(int i = 0; i < 10; ++i)
  {
    start = ros::Time::now();
    dist = df.getDistance(pick_point);
    t+=(ros::Time::now() - start).toSec();
  }

  ROS_ERROR("Sphere example, Background: %f", df.getGrid()->background());
  ROS_ERROR("Sphere example, Distance: %f", dist);
  ROS_ERROR("Sphere example, Gradient: %f %f %f", gradient(0), gradient(1), gradient(2));
  ROS_ERROR("Sphere example, Average Time Elapsed: %0.8f (sec)",t/10.0);

  // Write distance field to file
  df.writeToFile("test.vdb");
  ROS_ERROR("Sphere example, Bytes: %i", int(df.memUsage()));


// This tests the ClosestSurfacePoint which is about 10-30 times slower
// than just quering a distance grid, but the distace can be found when
// requesting a point outside the bandwidth. Though this does not return
// a negative distance when quering a point inside an object.
  openvdb::tools::ClosestSurfacePoint<openvdb::FloatGrid> csp;

  csp.initialize<openvdb::util::NullInterrupter>(*df.getGrid());
  std::vector<openvdb::Vec3R> points;
  std::vector<float> distance;

  points.push_back(openvdb::Vec3R(pick_point[0], pick_point[1], pick_point[2]));

  t=0;
  for(int i=0; i<10; i++)
  {
    distance.clear();
    start = ros::Time::now();
    csp.search(points, distance);
    dist = distance[0];
    t+=(ros::Time::now() - start).toSec();
  }

  ROS_ERROR("Closest Surface Point, Background: %f", df.getGrid()->background());
  ROS_ERROR("Closest Surface Point, Distance: %f", dist);
  ROS_ERROR("Closest Surface Point, Average Time Elapsed: %0.8f (sec)",t/10.0);


  // Test Openvdb Transforms
  openvdb::math::Mat4d mat;
  Eigen::Affine3d pose = robot_state.getGlobalLinkTransform("link_3");
  pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  distance_field::Affine3dToMat4dAffine(pose, mat);
  openvdb::math::Transform::Ptr linearTransform = openvdb::math::Transform::createLinearTransform(mat);
  openvdb::math::Transform::Ptr linearTransform1 = openvdb::math::Transform::createLinearTransform(mat);
  linearTransform1->preScale(0.1);

  openvdb::math::Vec3d ws = linearTransform1->indexToWorld(openvdb::math::Vec3d(0, 0, 0));
  openvdb::math::Vec3d ws2 = linearTransform1->indexToWorld(openvdb::math::Vec3d(1, 0, 0));

  openvdb::math::Transform::Ptr linearTransform2 = openvdb::math::Transform::createLinearTransform(0.1);
  openvdb::math::Transform::Ptr linearTransform3 = openvdb::math::Transform::createLinearTransform(1.0);
  linearTransform3->preScale(0.1);

  bool test = (*linearTransform2 == *linearTransform3);

  linearTransform2->preRotate(M_PI, openvdb::math::X_AXIS );

  // Test Openvdb Collision robot
  moveit::core::RobotState robot_state_vdb = robot_state;
  double background = 0.5;
  double voxel_size = 0.02;
  double exBandWidth = background/voxel_size;
  double inBandWidth = background/voxel_size;
  distance_field::CollisionRobotOpenVDB openvdb_robot(robot_model, voxel_size, background, exBandWidth, inBandWidth);
  t=0;
  req.gradient = true;

  ROS_ERROR("***********************************************************************************************************");
  ROS_ERROR("***************************************** Openvdb Collision Robot *****************************************");
  ROS_ERROR("***********************************************************************************************************");
  // For color information see: http://ascii-table.com/ansi-escape-sequences.php
  ROS_ERROR("\033[1;31m%20s %20s %10s %10s %10s %10s %10s %10s\033[0m", "Link Name #1", "Link Name #2", "Distance", "Grad. Norm", "Gradient X", "Gradient Y", "Gradient Z", "Avg Time");
  for(int i = 0; i < 200; ++i)
  {
    res.clear();
    start = ros::Time::now();
    openvdb_robot.distanceSelf(req, res, robot_state_vdb);
    t+=(ros::Time::now() - start).toSec();

    double norm = res.minimum_distance.gradient.norm();
    std::string code;
    if (norm < 0.98 && std::abs(res.minimum_distance.min_distance) < (background - 0.01))
      code = "\033[1;32m"; //bold; green
    else
      code = "\033[0;31m"; //norma; red

    ROS_ERROR("%s %20s %20s %10.2f %10.2f %10.2f %10.2f %10.2f %10.8f\033[0m",
              code.c_str(),
              res.minimum_distance.link_name[0].c_str(),
              res.minimum_distance.link_name[1].c_str(),
              res.minimum_distance.min_distance,
              res.minimum_distance.gradient.norm(),
              res.minimum_distance.gradient(0),
              res.minimum_distance.gradient(1),
              res.minimum_distance.gradient(2),
              t/(i + 1.0));

    robot_state_vdb.setToRandomPositions();
    robot_state_vdb.updateCollisionBodyTransforms();
  }

  // Write robot sdf to file
  openvdb_robot.writeToFile("test_robot.vdb");

  ROS_ERROR("Openvdb Collision Robot, Memory: %0.2f GB", openvdb_robot.memUsage()*1.0e-9);
  ROS_ERROR("***********************************************************************************************************");
  ROS_ERROR("***************************************** Openvdb Collision Robot *****************************************");
  ROS_ERROR("***********************************************************************************************************");

  return 0;
}