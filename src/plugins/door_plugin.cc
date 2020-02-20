// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define DEFAULT_OPEN_VEL -1.57
#define DEFAULT_CLOSE_VEL 1.57
#define DEFAULT_SLIDE_DISTANCE 0.711305

#define TYPE_FLIP_OPEN "flip"
#define TYPE_SLIDE_OPEN "slide"

#define DIRECTION_FLIP_CLOCKWISE "clockwise"
#define DIRECTION_FLIP_COUNTER_CLOCKWISE "counter_clockwise"
#define DIRECTION_SLIDE_LEFT "left"
#define DIRECTION_SLIDE_RIGHT "right"
#define DIRECTION_SLIDE_UP "up"
#define DIRECTION_SLIDE_DOWN "down"

#define CONTEXT_SPACE_X_RANGE 2.0       // in m
#define CONTEXT_SPACE_Y_RANGE 2.0
#define CONTEXT_SPACE_Z_RANGE 2.0


namespace gazebo
{
enum DoorType {FLIP, SLIDE};

class DoorPlugin : public ModelPlugin
{
private:
  physics::ModelPtr model;
  physics::LinkPtr doorLink;
  ignition::math::Pose3<double> currPose, currFpvPose;

  ignition::math::Vector3<double> cmd_vel;

  bool isActive;
  int activeDoors[100];
  DoorType type;

  int door_ref_num;
  std::string door_type, door_model_name, door_direction, model_domain_space;
  double max_trans_dist, maxPosX, maxPosY, maxPosZ, minPosX, minPosY, minPosZ;

  ros::NodeHandle* rosNode;
  transport::NodePtr gazeboNode;
  event::ConnectionPtr updateConnection;

  transport::SubscriberPtr subFpvPose, subGzRequest;
  ros::Subscriber sub, sub_active;

public:
  DoorPlugin()
  {
    std::string name = "door_plugin_node";
    int argc = 0;
    ros::init(argc, NULL, name);
  }
  ~DoorPlugin()
  {
    delete rosNode;
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    establishLinks(_parent);
    determineDoorType(_sdf);
    determineDoorDirection(_sdf);
    determineModelDomain(_sdf);
    determineConstraints(_sdf);
    initVars();
  }

  void OnUpdate()
  {
    ros::spinOnce();
    updateLinkVel();
    applyConstraints();
  }

private:
  void determineDoorType(sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("door_type"))
    {
      ROS_WARN("Door Type not specified. Defaulting to 'flip'");
      door_type = "flip";
    }
    else
    {
      door_type = _sdf->GetElement("door_type")->Get<std::string>();
    }

    if (door_type.compare(TYPE_SLIDE_OPEN) == 0)
    {
      type = SLIDE;
    }
    else
    {
      type = FLIP;
    }
  }

  void determineDoorDirection(sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("door_direction"))
    {
      if (type == FLIP)
      {
        ROS_WARN("Door direction not specified in the plugin reference. Defaulting to 'clockwise'");
        door_direction = DIRECTION_FLIP_CLOCKWISE;
      }
      else if (type == SLIDE)
      {
        ROS_WARN("Door direction not specified in the plugin reference. Defaulting to 'left'");
        door_direction = DIRECTION_SLIDE_LEFT;
      }
    }
    else
    {
      door_direction = _sdf->GetElement("door_direction")->Get<std::string>();
      checkDirectionValidity();
    }
  }

  void determineConstraints(sdf::ElementPtr _sdf)
  {
    if (type == SLIDE)
    {
      if (!_sdf->HasElement("max_trans_dist"))
      {
        ROS_WARN("Max Translation Distance for sliding door not specified in the plugin reference. Defaulting to '0.711305' m");
        max_trans_dist = DEFAULT_SLIDE_DISTANCE;
      }
      else
      {
        max_trans_dist = _sdf->GetElement("max_trans_dist")->Get<double>();
      }
    }
  }

  void checkDirectionValidity()
  {
    if (type == FLIP)
    {
      if (door_direction != DIRECTION_FLIP_CLOCKWISE &&
          door_direction != DIRECTION_FLIP_COUNTER_CLOCKWISE)
      {
        ROS_WARN("Invalid door direction specified. Only two states possible: 'clockwise' OR 'counter_clockwise'. Defaulting to 'clockwise'");
        door_direction = DIRECTION_FLIP_CLOCKWISE;
      }
    }
    else if (type == SLIDE)
    {
      if (door_direction != DIRECTION_SLIDE_LEFT &&
          door_direction != DIRECTION_SLIDE_RIGHT &&
          door_direction != DIRECTION_SLIDE_UP &&
          door_direction != DIRECTION_SLIDE_DOWN)
      {
        ROS_WARN("Invalid door direction specified. Only four states possible: 'left' OR 'right' OR 'up' or 'down'. Defaulting to 'left'");
        door_direction = DIRECTION_SLIDE_LEFT;
      }
    }
  }

  void determineModelDomain(sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("model_domain_space"))
    {
      ROS_WARN("Model Domain Space not specified in the plugin reference. Defaulting to 'door_'");
      model_domain_space = "door_";
    }
    else
    {
      model_domain_space = _sdf->GetElement("model_domain_space")->Get<std::string>();
    }

    ROS_INFO("Door '%s' initialized - Type: %s, Direction: %s, Domain Space: %s\n", door_type.c_str(), door_model_name.c_str(), door_direction.c_str(), model_domain_space.c_str());
  }

  void initVars()
  {
    isActive = false;

    // find the elevator reference number
    std::string door_ref_num_str = door_model_name;
    replaceSubstring(door_ref_num_str, model_domain_space, "");
    door_ref_num = atoi(door_ref_num_str.c_str());

    if (type == SLIDE)
    {
      // compute slide constraints
      double spawnPosX = model->WorldPose().Pos().X();
      minPosX = door_direction != DIRECTION_SLIDE_RIGHT ? spawnPosX - max_trans_dist : spawnPosX;
      maxPosX = door_direction != DIRECTION_SLIDE_RIGHT ? spawnPosX : spawnPosX + max_trans_dist;

      double spawnPosY = model->WorldPose().Pos().Y();
      minPosY = door_direction != DIRECTION_SLIDE_RIGHT ? spawnPosY - max_trans_dist : spawnPosY;
      maxPosY = door_direction != DIRECTION_SLIDE_RIGHT ? spawnPosY : spawnPosY + max_trans_dist;

      double spawnPosZ = model->WorldPose().Pos().Z();
      minPosZ = door_direction != DIRECTION_SLIDE_UP ? spawnPosZ - max_trans_dist : spawnPosZ;
      maxPosZ = door_direction != DIRECTION_SLIDE_UP ? spawnPosZ : spawnPosZ + max_trans_dist;
    }
  }

  void establishLinks(physics::ModelPtr _parent)
  {
    model = _parent;
    doorLink = model->GetLink("door");
    door_model_name = model->GetName();

    rosNode = new ros::NodeHandle("");

    sub = rosNode->subscribe<geometry_msgs::Twist>("/door_controller/command", 1000, &DoorPlugin::cmd_ang_cb, this );
    sub_active = rosNode->subscribe<std_msgs::UInt32MultiArray>("/door_controller/active", 1000, &DoorPlugin::active_doors_cb, this);

    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DoorPlugin::OnUpdate, this));
  }

  void cmd_ang_cb(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (isActive)
    {
      if (type == FLIP)
      {
        setAngularVel(msg->angular.z);
        ROS_INFO("Door '%s' - Angular z: [%f]", door_model_name.c_str(), msg->angular.z);
      }
      else if (type == SLIDE)
      {
        setLinearVel(msg->linear.x, msg->linear.y, msg->linear.z);
        ROS_INFO("Door '%s' - Linear x: [%f], y: [%f], z: [%f]", door_model_name.c_str(), msg->linear.x, msg->linear.y, msg->linear.z);
      }
    }
  }

  void updateLinkVel()
  {
    if (type == FLIP)
    {
      doorLink->SetAngularVel(cmd_vel);
    }
    else if (type == SLIDE)
    {
      doorLink->SetLinearVel(cmd_vel);
    }
  }

  void applyConstraints()
  {
    if (type == SLIDE)
    {
      const double currDoorPosX = model->WorldPose().Pos().X();
      const double currDoorPosY = model->WorldPose().Pos().Y();
      const double currDoorPosZ = model->WorldPose().Pos().Z();

      ignition::math::Pose3<double> constrainedPose;

      if (currDoorPosX > maxPosX)
      {
        constrainedPose.Pos().X(maxPosX);
      }
      else if (currDoorPosX < minPosX)
      {
        constrainedPose.Pos().X(minPosX);
      }
      else
      {
        constrainedPose.Pos().X(currDoorPosX);
      }

      if (currDoorPosY > maxPosY)
      {
        constrainedPose.Pos().Y(maxPosY);
      }
      else if (currDoorPosY < minPosY)
      {
        constrainedPose.Pos().Y(minPosY);
      }
      else
      {
        constrainedPose.Pos().Y(currDoorPosY);
      }

      if (currDoorPosZ > maxPosZ)
      {
        constrainedPose.Pos().Z(maxPosZ);
      }
      else if (currDoorPosZ < minPosZ)
      {
        constrainedPose.Pos().Z(minPosZ);
      }
      else
      {
        constrainedPose.Pos().Z(currDoorPosZ);
      }

      constrainedPose.Rot().W(model->WorldPose().Rot().W());
      constrainedPose.Rot().X(model->WorldPose().Rot().X());
      constrainedPose.Rot().Y(model->WorldPose().Rot().Y());
      constrainedPose.Rot().Z(model->WorldPose().Rot().Z());

      model->SetWorldPose(constrainedPose);
    }
  }

  void setAngularVel(const double rot_z)
  {
    cmd_vel = ignition::math::Vector3<double>();

    if (door_direction.compare(DIRECTION_FLIP_CLOCKWISE) == 0)
    {
      cmd_vel.Z(rot_z);
    }
    else
    {
      cmd_vel.Z(-rot_z);
    }
  }

  void setLinearVel(const double x = 0.0, const double y = 0.0, const double z = 0.0)
  {
    cmd_vel = ignition::math::Vector3<double>();

    if (door_direction == DIRECTION_SLIDE_LEFT)
    {
      cmd_vel.X(-x);
      cmd_vel.Y(-y);
    }
    else if (door_direction == DIRECTION_SLIDE_RIGHT)
    {
      cmd_vel.X(x);
      cmd_vel.Y(y);
    }
    else if (door_direction == DIRECTION_SLIDE_UP)
    {
      cmd_vel.Z(-z);
    }
    else if (door_direction == DIRECTION_SLIDE_DOWN)
    {
      cmd_vel.Z(z);
    }
  }

  void active_doors_cb(const std_msgs::UInt32MultiArray::ConstPtr& array)
  {
    isActive = false;

    int i = 0;

    for (std::vector<uint32_t>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
      activeDoors[i] = *it;
      i++;

      if (*it == door_ref_num)
      {
        isActive = true;
      }
    }
  }

  std::string replaceSubstring(std::string &s, std::string toReplace, std::string replaceWith)
  {
    return(s.replace(s.find(toReplace), toReplace.length(), replaceWith));
  }

  // Deprecated function:
  std::vector<std::string> parseTopicStr(std::string bot_pose_topics_str)
  {
    std::vector<std::string> bot_pose_topic_list;

    // parse csv-style input (also remove whitespace):
    std::string::iterator end_pos = std::remove(bot_pose_topics_str.begin(), bot_pose_topics_str.end(), ' ');
    bot_pose_topics_str.erase(end_pos, bot_pose_topics_str.end());

    std::istringstream ss(bot_pose_topics_str);
    std::string token;

    while (std::getline(ss, token, ','))
    {
      bot_pose_topic_list.push_back(token.c_str());
    }

    return bot_pose_topic_list;
  }
};

GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)
}
