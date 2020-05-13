// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/server/simple_action_server.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/GenerateGraspsAction.h>
#include <moveit_simple_grasps/GraspGeneratorOptions.h>


// Baxter specific properties
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/custom_environment2.h>

namespace moveit_simple_grasps
{

  bool graspGeneratorOptions2Inner(
          const moveit_simple_grasps::GraspGeneratorOptions &options,
          grasp_axis_t &axis,
          grasp_direction_t &direction,
          grasp_rotation_t &rotation)
  {
    switch(options.grasp_axis)
    {
     case GraspGeneratorOptions::GRASP_AXIS_X:
        axis = X_AXIS;
        break;
     case GraspGeneratorOptions::GRASP_AXIS_Y:
        axis = Y_AXIS;
        break;
     case GraspGeneratorOptions::GRASP_AXIS_Z:
        axis = Z_AXIS;
        break;
     default:
        assert(false);
        break;
    }

    switch(options.grasp_direction)
    {
     case GraspGeneratorOptions::GRASP_DIRECTION_UP:
        direction = UP;
        break;
     case GraspGeneratorOptions::GRASP_DIRECTION_DOWN:
        direction = DOWN;
        break;
     default:
        assert(false);
        break;
    }

    switch(options.grasp_rotation)
    {
     case GraspGeneratorOptions::GRASP_ROTATION_FULL:
        rotation = FULL;
        break;
     case GraspGeneratorOptions::GRASP_ROTATION_HALF:
        rotation = HALF;
        break;
     default:
        assert(false);
        break;
    }
    return true;
  }

  class GraspGeneratorServer
  {
  private:
    // A shared node handle
    ros::NodeHandle nh_;

    // Action server
    actionlib::SimpleActionServer<moveit_simple_grasps::GenerateGraspsAction> as_;
    moveit_simple_grasps::GenerateGraspsResult result_;

    // Grasp generator
    moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

    // class for publishing stuff to rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // robot-specific data for generating grasps
    moveit_simple_grasps::GraspData grasp_data_;

    // which arm are we using
    std::string side_;
    std::string planning_group_name_;

  public:

    // Constructor
    GraspGeneratorServer(const std::string &name, const std::string &side)
      : nh_("~")
      , as_(nh_, name, boost::bind(&moveit_simple_grasps::GraspGeneratorServer::executeCB, this, _1), false)
      , side_(side)
      , planning_group_name_("arm")
//      , planning_group_name_(side_+"_arm")

    {
      // ---------------------------------------------------------------------------------------------
      // Load grasp data specific to our robot
      if (!grasp_data_.loadRobotGraspData(nh_, side_))
        ros::shutdown();

      // ---------------------------------------------------------------------------------------------
      // Load the Robot Viz Tools for publishing to Rviz
      visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_));
      visual_tools_->setLifetime(120.0);
      visual_tools_->setMuted(false);
      visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

      // ---------------------------------------------------------------------------------------------
      // Load grasp generator
      simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
      as_.start();
    }

    void executeCB(const moveit_simple_grasps::GenerateGraspsGoalConstPtr &goal)
    {
      // ---------------------------------------------------------------------------------------------
      // Remove previous results
      result_.grasps.clear();

      // ---------------------------------------------------------------------------------------------
      // Set object width and generate grasps
      grasp_data_.object_size_ = goal->width;

      // Generate grasps for all options that were passed
      grasp_axis_t axis;
      grasp_direction_t direction;
      grasp_rotation_t rotation;
      for(size_t i=0; i<goal->options.size(); ++i)
      {
        graspGeneratorOptions2Inner(goal->options[i], axis, direction, rotation);
        simple_grasps_->generateAxisGrasps(goal->pose, axis, direction, rotation, 0, grasp_data_, result_.grasps);
      }
      // fallback behaviour, generate default grasps when no options were passed
      if(goal->options.empty())
      {
        simple_grasps_->generateBlockGrasps(goal->pose, grasp_data_, result_.grasps);
      }

      // ---------------------------------------------------------------------------------------------
      // Publish results
      as_.setSucceeded(result_);
    }

  };
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_server");
  moveit_simple_grasps::GraspGeneratorServer grasp_generator_server("generate", "gripper");
  ros::spin();
  return 0;
}
