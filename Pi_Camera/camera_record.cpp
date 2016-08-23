/*********************************
* Starts and stops recording with rosbag
* Written by: Meenu Goswami
*
* Subscriber to start recording and stop recording
*      message type: std_msgs::String
*      message value: "1" or "0"
*                   1 - to start recording
*                   0 - to stop recording 
* The bag file is created in the directory this program 
* is run from. 
***********************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h> 
#include <sys/types.h>
#include <signal.h>

pid_t PID ; // process id of the process forked
bool started = false; // check if recording is started

// callback function
void image_recorderCallback(const std_msgs::String::ConstPtr& msg)
{
  
  if ( strcmp((const char* ) msg->data.c_str(), "1") == 0 && !started)
  { // start the recording process

    started = true;
    ROS_INFO("recording starting.");
    PID = fork();

    if (PID < 0){
       started = false;
       ROS_INFO("Error: Creating a child process for rosbag");
    }

    else if(PID == 0) {

       // Use setpgid in the child to set its Group PID equal to its own PID.
       // The parent then can kill(-pid,...) to signal the entire group.
       setpgid(0, 0);

       // looks for rosbag from the directories specified by PATH env. var.
       execlp("rosbag", "rosbag", "record", "/cmd_vel", "/tf", "/right_cam/image/compressed", (char *)0);
    }
  }

  else if ( strcmp((const char* ) msg->data.c_str(), "0") == 0 && started)
  {
     // kill the group process, SIGINT will do a clean exit of rosbag 
     kill(-PID, SIGINT);  
     started = false;
     ROS_INFO("recording stopped.");
  }
}

int main(int argc, char **argv)
{

  // node name
  ros::init(argc, argv, "image_recorder");

  // node handler
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("image_record", 50, image_recorderCallback);

  ros::spin();

  return 0;
} // main()
