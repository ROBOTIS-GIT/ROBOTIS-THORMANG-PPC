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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <laser_assembler/AssembleScans2.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Time           g_lidar_move_start_time;

ros::Publisher      g_point_cloud2_pub;

ros::Subscriber     g_lidar_turn_start_sub;
ros::Subscriber     g_lidar_turn_end_sub;

ros::ServiceClient  g_assemble_chest_laser_client;

typedef pcl::PointXYZ PointT;

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void assembleLaserScans(ros::Time before_time, ros::Time end_time)
{
  ros::Time now = ros::Time::now();

  laser_assembler::AssembleScans2 service;
  service.request.begin = before_time;
  service.request.end = end_time;

  if (g_assemble_chest_laser_client.call(service))
  {
    ros::Time assemble_time = ros::Time::now();
    sensor_msgs::PointCloud2 assembler_output = service.response.cloud;
    if (assembler_output.data.size() == 0)
    {
      // ROS_INFO("No scan data");
      return;
    }

    ROS_INFO("  ---  publish pointcloud data!!  ---  %f", (ros::Time::now() - assemble_time).toSec());

    g_point_cloud2_pub.publish(assembler_output);
  }
}

void lidarTurnCallBack(const std_msgs::String::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  if (msg->data == "start")
  {
    g_lidar_move_start_time = now;
  }
  else if (msg->data == "end")
  {
    // assemble laser
    assembleLaserScans(g_lidar_move_start_time, now);
    g_lidar_move_start_time = now;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_lidar_assembler");
  ros::NodeHandle nh;

  // Add your ros communications here.
  g_lidar_turn_end_sub          = nh.subscribe("/robotis/sensor/move_lidar", 1, &lidarTurnCallBack);
  g_point_cloud2_pub            = nh.advertise<sensor_msgs::PointCloud2>("/robotis/sensor/assembled_scan", 0);
  g_assemble_chest_laser_client = nh.serviceClient<laser_assembler::AssembleScans2>("/robotis/sensor/service/assemble_scans2");

  g_lidar_move_start_time = ros::Time::now();

  ros::spin();

  return 0;
}
