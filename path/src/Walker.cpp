/***************************************************************************
 MIT License
 Copyright © 2019 Raj Shinde
 
 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 Files (the “Software”), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge,
 publish, distribute, sublicense, and/or sell copies of the Software,
 and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:
 The above copyright notice and this permission notice shall be included 
 in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
 OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 ***************************************************************************/

/**
 *  @copyright MIT License, © 2019 Raj Shinde
 *  @file    Walker.cpp
 *  @author  Raj Shinde
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief   walker algorithm
 *  @section DESCRIPTION
 *  Uses laser data to walk and avoid obstacles
 */

#include <iostream>
// #include <fstream>
#include <string>
#include <Walker.hpp>
#include <time.h>


Walker::Walker() {
  // Initialize variables in constructors
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

Walker::~Walker() {
// Destroy Objects
vel.publish(msg);
}

std::vector<std::pair<int,int>> Walker::readTextFile() {
  std::string line;
  std::ifstream infile;
  std::ifstream count;
  std::vector< std::pair <int,int> > path; 
  std::string a;
  std::string temp; 
  int p;
  int q;

  infile.open ("/home/rajshinde/Documents/mew/src/path/src/velocity.txt");
int coun=1;
  do{
          getline(infile,line);
            p=std::stoi(line.substr(1,2));
            ROS_INFO_STREAM("Line="<<p);
            if(p>9)
            {
              q=std::stoi(line.substr(4,2));
              ROS_INFO_STREAM("Line="<<q);
            }
            else{
            q=std::stoi(line.substr(3,2));
            ROS_INFO_STREAM("Line="<<q);
            }
            path.push_back(std::make_pair(p,q));
    coun++;
       }while(coun!=19);
  infile.close();
  return path;
}

/**
 * @brief function to navigate robot
 * @param None
 * @return None
 */
void Walker::navigate() {
ROS_INFO_STREAM("In");
std::vector< std::pair <int,int> > path=readTextFile();
ROS_INFO_STREAM("In");

ros::NodeHandle n;
 auto vel = n.advertise<geometry_msgs::Twist>
         ("/cmd_vel_mux/input/navi", 1000);

  ros::Rate loop_rate(10);
  int i=0;
  int s=1;
  ros::Duration(10.0).sleep(); 
  while (ros::ok()) {
    double axil=0.354;
    double wheel=0.038;
    float time_interval=2;
    double angular_velocity, linear_velocity,curvature,inner_arc,outer_arc, left_velocity,right_velocity;
    ROS_INFO_STREAM("path1="<<path[i].first);
    ROS_INFO_STREAM("path2="<<path[i].second);
    left_velocity=(path[i].first*9.549*wheel*2*3.141)/(60);
    right_velocity=(path[i].second*9.549*wheel*2*3.141)/(60);

    // linear_velocity=0.5*(left_velocity+right_velocity);
   
    if(left_velocity<right_velocity)
    {
    inner_arc=left_velocity*time_interval;
    outer_arc=right_velocity*time_interval;
    s=-1;
    curvature=axil*(inner_arc/(outer_arc - inner_arc));
    angular_velocity=(s*outer_arc)/((curvature+axil)*time_interval);
    linear_velocity=s*angular_velocity*(curvature+(axil/2));
    }
    else if(left_velocity>right_velocity)
    {
    inner_arc=right_velocity*time_interval;
    outer_arc=left_velocity*time_interval;
    s=1;
    curvature=axil*(inner_arc/(outer_arc - inner_arc));
    angular_velocity=(s*outer_arc)/((curvature+axil)*time_interval);
    linear_velocity=s*angular_velocity*(curvature+(axil/2));
    }
    else if(left_velocity==right_velocity)
    {
    inner_arc=right_velocity*time_interval;
    outer_arc=left_velocity*time_interval;
    linear_velocity=inner_arc/time_interval;
    }
  
  
    msg.linear.x = linear_velocity;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular_velocity;
    vel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("Itr"<<i);
     ROS_INFO_STREAM("Vel"<<linear_velocity);
     ROS_INFO_STREAM("angular="<<angular_velocity);
    i++;

    if(i==path.size())
    {
     msg.linear.x = 0.0;
     msg.linear.y = 0.0;
     msg.linear.z = 0.0;
     msg.angular.x = 0.0;
     msg.angular.y = 0.0;
     msg.angular.z = 0.0;
     vel.publish(msg);
     ros::spinOnce();
     ros::shutdown();
    }
    ros::Duration(2.0).sleep(); 
  }
}

