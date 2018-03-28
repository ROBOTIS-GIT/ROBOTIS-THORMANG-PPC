/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * main.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: Jay Song
 */

#include "thormang3_walking_demo/thormang3_walking_demo.h"

ros::Subscriber g_demo_command_sub;

bool is_init_pose = false;

void demoCommandCallback(const std_msgs::String::ConstPtr& msg)
{

  ROS_INFO_STREAM("[Demo]  : receive [" << msg->data << "] msg " );

  if(msg->data == "ini_pose")
  {
    ROS_INFO("demo 1: go to initial pose");
    moveToInitPose();
    is_init_pose = true;
    ROS_INFO("[Demo]  : please wait 5 seconds");
  }
  else if ( msg->data == "set_mode")
  {
    ROS_INFO("demo 2: set walking control mode");
    setCtrlModule();
  }
  else if( msg->data == "forward" )
  {
    ROS_INFO("demo 4: forward walking");
    walkForward();
  }
  else if( msg->data == "backward" )
  {
    ROS_INFO("demo 5: backward walking");
    walkBackward();
  }
  else if( msg->data == "balance_on" )
  {
    ROS_INFO("demo 3: balance enable");
    setBalanceOn();
  }
  else if( msg->data == "balance_off" )
  {
    ROS_INFO("demo 3: balance disable");
    setBalanceOff();
  }
  else {
    ROS_ERROR("Invalid Command!!!");
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thormang3_walking_demo");

  ROS_INFO("ROBOTIS THORMANG3 Walking Simple Demo");

  initialize();

  ros::NodeHandle nh;
  g_demo_command_sub = nh.subscribe("/robotis/walking_demo/command", 10, demoCommandCallback);

  ros::spin();
  return 0;
}

