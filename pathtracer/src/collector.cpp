  /************************************************************************
BSD 3-Clause License

Copyright (c) 2019, Raj Shinde
Copyright (c) 2019, Prasheel Renkuntla
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
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
 *************************************************************************/

/**
 *  @copyright BSD 3-Clause License 
 *  @copyright Copyright © 2019 Raj Shinde, Prasheel Renkuntla
 *  @file    collector.cpp
 *  @author  Prasheel Renkuntla
 *  @author  Raj Shinde
 *  @date    12/09/2019
 *  @version 3.0
 *  @brief   Final Project - ecobot (A trash Collecting Robot)
 *  @section Implemention of collection task
 */

#include <ros/ros.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include "spawnCollect.hpp"
#include "randomizer.hpp"
#include "collector.hpp"
#include <string>
#include <time.h>

typedef actionlib::SimpleActionClient
<move_base_msgs::MoveBaseAction> MoveBaseClient;

Collector::Collector() {
}
Collector::~Collector() {
}

std::vector<std::pair<int,int>> Collector::readTextFile() {
  std::string line;
  std::ifstream infile;
  std::ifstream count;
  std::vector< std::pair <int,int> > path; 
  std::string a;
  std::string temp; 
  double p;
  double q;

  infile.open ("/home/rajshinde/Documents/mew/src/pathtracer/data/maps/waypoints.txt");
int coun=1;
  do{
          getline(infile,line);
            p=std::stoi(line.substr(2,1));
            p=p+(0.01*std::stoi(line.substr(4,2)));
            if(line.substr(1,1)=="-")
            {
              p=-1*p;
            }
            ROS_INFO_STREAM("Line="<<p);

            q=std::stoi(line.substr(8,1));
            q=q+(0.01*std::stoi(line.substr(10,2)));
            if(line.substr(7,1)=="-")
            {
              q=-1*q;
            }
            ROS_INFO_STREAM("Line="<<q);

            path.push_back(std::make_pair(p,q));
      coun++;
       }while(coun!=53);
  infile.close();
  return path;
}

bool Collector::collector() {

std::vector< std::pair <int,int> > path=readTextFile();
double xn=0;
double yn=0;

  // tell the action client  to spin a thread
  MoveBaseClient ac("move_base", true);

  // wait for action server
  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  int i=1;
  while(i!=path.size())
  {
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -path[i-1].second;
  goal.target_pose.pose.position.y = 2-path[i-1].first;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

// geometry_msgs::Pose g_pose;
//             g_pose.position.x=path[i-1].first;
//             g_pose.position.y=path[i-1].second;
//             g_pose.position.z=0.0;
//             g_pose.orientation.x=0.0;
//             g_pose.orientation.y=0.0;
//             g_pose.orientation.z=0.0;
//             g_pose.orientation.w=1.0;
//    tf::TransformListener part_tf_listener_;
//   geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
//   StampedPose_in.header.frame_id = "/world";
//   StampedPose_in.pose = g_pose;
//   part_tf_listener_.transformPose("/map",StampedPose_in,StampedPose_out);

//   goal.target_pose.pose.position.x = StampedPose_out.pose.position.x;
//   goal.target_pose.pose.position.y = StampedPose_out.pose.position.y;
//   goal.target_pose.pose.position.z = 0.0;
//   goal.target_pose.pose.orientation.w = 1.0;
//   ROS_INFO_STREAM("map x="<<StampedPose_out.pose.position.x);
//   ROS_INFO_STREAM("map y="<<StampedPose_out.pose.position.y);

  ROS_INFO_STREAM("Sending goal");
  ac.sendGoal(goal);
  
  // ac.waitForResult(ros::Duration(3.0));
  // ros::Duration(5.0).sleep(); 

  i++;
  while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
  }
  // ros::Duration(10.0).sleep(); 

  ROS_INFO_STREAM("Waypoint");
  }





return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  Collector trash;
  trash.collector();
  return 0;
}
